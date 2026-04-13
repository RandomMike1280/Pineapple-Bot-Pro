package com.example.pineapplerobotpro

import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress

// ============================================================================
// Robot UDP Bridge — Handles robot↔phone communication over UDP
// ============================================================================
// Protocol:
//   Robot → Phone:  H:<robot_id>:<robot_ip>   (HELLO beacon, every 3s)
//                   P:<timestamp>              (PING)
//                   S:<x>:<y>:<queue_len>:<drift>  (STATUS)
//   Phone → Robot:  C:<timestamp_ms>:<x_mm>:<y_mm>:<angle_deg>  (CAM observation)
//                   Q:<orig_timestamp>          (PONG reply)
//
// Handshake: Robot connects to phone hotspot, broadcasts HELLO on port 4210.
//            Phone listens on 4210, picks up robot IP from packet source,
//            then sends camera observations back to that IP on same port.
// ============================================================================

data class ConnectedRobot(
    val id: String,
    val address: InetAddress,
    val port: Int,
    val lastSeenMs: Long
)

data class TapTargetState(
    val robotId: String,
    val targetXmm: Float,
    val targetYmm: Float,
    val holdAngleDeg: Float,
    val arrivalToleranceMm: Float = 20f,
    val angleToleranceDeg: Float = 2f,
    val active: Boolean = true,
    val lastCommandTimeMs: Long = 0L
)

data class HeadingLockState(
    val robotId: String,
    val targetAngleDeg: Float,
    val angleToleranceDeg: Float = 1.25f,
    val stableFrameCount: Int = 0,
    val active: Boolean = true,
    val lastCommandTimeMs: Long = 0L
)

class RobotUdpBridge(private val port: Int = 4210) {

    private val _robots = MutableStateFlow<Map<String, ConnectedRobot>>(emptyMap())
    val robots: StateFlow<Map<String, ConnectedRobot>> = _robots.asStateFlow()

    private val _logs = MutableStateFlow<List<String>>(emptyList())
    val logs: StateFlow<List<String>> = _logs.asStateFlow()

    private var socket: DatagramSocket? = null
    private var listenerJob: Job? = null
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())
    private val motionCommandEpoch = mutableMapOf<String, Long>()

    private fun nextMotionCommandEpoch(robotId: String): Long = synchronized(motionCommandEpoch) {
        val next = (motionCommandEpoch[robotId] ?: 0L) + 1L
        motionCommandEpoch[robotId] = next
        next
    }

    private fun currentMotionCommandEpoch(robotId: String): Long = synchronized(motionCommandEpoch) {
        motionCommandEpoch[robotId] ?: 0L
    }

    private fun invalidateMotionCommands(robotId: String) {
        synchronized(motionCommandEpoch) {
            motionCommandEpoch[robotId] = (motionCommandEpoch[robotId] ?: 0L) + 1L
        }
    }

    private fun log(msg: String) {
        val time = System.currentTimeMillis() % 100000  // last 5 digits for brevity
        val entry = "[${time}] $msg"
        _logs.value = (listOf(entry) + _logs.value).take(30)
    }

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    fun start() {
        if (listenerJob?.isActive == true) return
        listenerJob = scope.launch {
            try {
                socket?.close()
                socket = DatagramSocket(port).apply { soTimeout = 1000 }
                log("Listening on UDP :$port")

                val buf = ByteArray(512)
                while (isActive) {
                    try {
                        val pkt = DatagramPacket(buf, buf.size)
                        socket?.receive(pkt)
                        val msg = String(pkt.data, 0, pkt.length, Charsets.US_ASCII).trim()
                        handleIncoming(msg, pkt.address, pkt.port)
                    } catch (_: java.net.SocketTimeoutException) {
                        // expected
                    }
                }
            } catch (e: Exception) {
                log("Listener error: ${e.message}")
            }
        }
    }

    fun stop() {
        listenerJob?.cancel()
        socket?.close()
        socket = null
    }

    // ------------------------------------------------------------------
    // Incoming message handling
    // ------------------------------------------------------------------

    private fun handleIncoming(msg: String, senderAddr: InetAddress, senderPort: Int) {
        val parts = msg.split(":")
        if (parts.isEmpty()) return

        when (parts[0]) {
            // HELLO: H:<robot_id>[:<robot_ip>]
            "H" -> {
                val robotId = if (parts.size >= 2) parts[1] else "?"
                registerRobot(robotId, senderAddr, port)
                log("HELLO from Robot $robotId @ ${senderAddr.hostAddress}")
            }
            // REGISTER: R:<robot_id>:<caps>[:<robot_ip>]
            "R" -> {
                val robotId = if (parts.size >= 2) parts[1] else "?"
                registerRobot(robotId, senderAddr, port)
                log("REGISTER from Robot $robotId @ ${senderAddr.hostAddress}")
            }
            // PING: P:<timestamp> — auto-reply with PONG including phone time
            "P" -> {
                if (parts.size >= 2) {
                    val phoneNow = System.currentTimeMillis()
                    val pong = "Q:${parts[1]}:$phoneNow"
                    sendRaw(senderAddr, port, pong)
                }
            }
            // STATUS: S:<x>:<y>:<queue_len>:<drift>
            "S" -> {
                // silently ignore
            }
            else -> {
                log("RX: $msg from ${senderAddr.hostAddress}")
            }
        }
    }

    private fun registerRobot(id: String, address: InetAddress, port: Int) {
        val robot = ConnectedRobot(
            id = id,
            address = address,
            port = port,
            lastSeenMs = System.currentTimeMillis()
        )
        _robots.value = _robots.value + (id to robot)
    }

    // ------------------------------------------------------------------
    // Outgoing: Send camera observation to a specific robot
    // ------------------------------------------------------------------

    /**
     * Send a camera position/angle observation to the robot.
     * @param robotId  "A" or "B"
     * @param xMm      X position in mm (field coordinates)
     * @param yMm      Y position in mm (field coordinates)
     * @param angleDeg Heading angle in degrees (0=right, CCW positive)
     */
    fun sendCameraObservation(robotId: String, xMm: Float, yMm: Float, angleDeg: Float, captureTimeMs: Long) {
        val robot = _robots.value[robotId] ?: return
        val msg = "C:$captureTimeMs:${"%.1f".format(xMm)}:${"%.1f".format(yMm)}:${"%.1f".format(angleDeg)}"
        scope.launch {
            sendRaw(robot.address, robot.port, msg)
        }
    }

    /**
     * Send a camera observation to ALL connected robots whose marker is visible.
     */
    fun broadcastCameraObservations(observations: Map<String, Triple<Float, Float, Float>>, captureTimeMs: Long) {
        for ((robotId, obs) in observations) {
            sendCameraObservation(robotId, obs.first, obs.second, obs.third, captureTimeMs)
        }
    }

    // ------------------------------------------------------------------
    // Outgoing: Send arbitrary protocol command to a robot
    // ------------------------------------------------------------------

    /**
     * Send a raw protocol command string to a specific robot.
     * @param robotId  "A" or "B"
     * @param cmd      Full command string, e.g. "M:0:200:1:0"
     */
    fun sendCommand(robotId: String, cmd: String) {
        val robot = _robots.value[robotId] ?: return
        if (cmd == "A") {
            invalidateMotionCommands(robotId)
        }
        scope.launch {
            sendRaw(robot.address, robot.port, cmd)
            log("TX→$robotId: $cmd")
        }
    }

    /**
     * Send ABORT first to clear any stale queued commands, then send the new command.
     * This ensures the robot only ever executes the latest instruction.
     */
    private fun sendFreshCommand(robotId: String, cmd: String) {
        val robot = _robots.value[robotId] ?: return
        val epoch = nextMotionCommandEpoch(robotId)
        scope.launch {
            sendRaw(robot.address, robot.port, "A")
            delay(20)  // small gap so ABORT is processed first
            if (currentMotionCommandEpoch(robotId) != epoch) return@launch
            sendRaw(robot.address, robot.port, cmd)
            log("TX→$robotId: $cmd")
        }
    }

    // ==================================================================
    // Phone-driven square test
    // ==================================================================
    // The phone is the brain.  Every camera frame it:
    //   1. Checks the robot's camera-observed position & heading
    //   2. Decides what the robot should do (rotate / translate)
    //   3. Sends short duration-based move commands (~500ms bursts)
    //   4. Waits for the burst to finish, then re-evaluates
    // This avoids any reliance on dead-reckoning accuracy.
    // ==================================================================

    private val MOVE_BURST_MS = 400L      // how long each move command lasts
    private val ROTATE_BURST_MS = 120L    // short camera-guided rotation burst
    private val MOVE_COMMAND_COOLDOWN_MS = 500L
    private val ROTATE_COMMAND_COOLDOWN_MS = 140L
    private val HEADING_LOCK_CONFIRM_FRAMES = 4

    // Tap-target: only stop-and-rotate when heading error exceeds this.
    // Small drifts are handled by the firmware's built-in angle P-controller
    // during waypoint translation (Kp=0.03, clamped ±0.3).
    private val TT_HEADING_CORRECTION_THRESHOLD_DEG = 15f

    fun startTapTarget(
        robotId: String,
        targetXmm: Float,
        targetYmm: Float,
        holdAngleDeg: Float
    ) {
        _robots.value[robotId] ?: return
        val headingState = _headingLockState.value
        if (headingState?.active == true) {
            _headingLockState.value = headingState.copy(active = false)
            if (headingState.robotId != robotId) {
                sendCommand(headingState.robotId, "A")
            }
        }
        val tapState = _tapTargetState.value
        if (tapState?.active == true && tapState.robotId != robotId) {
            _tapTargetState.value = tapState.copy(active = false)
            sendCommand(tapState.robotId, "A")
        }
        sendCommand(robotId, "A")
        _tapTargetState.value = TapTargetState(
            robotId = robotId,
            targetXmm = targetXmm,
            targetYmm = targetYmm,
            holdAngleDeg = holdAngleDeg,
            lastCommandTimeMs = 0L
        )
        log("TT: target Robot $robotId → (${"%.0f".format(targetXmm)},${"%.0f".format(targetYmm)}) hold ${"%.1f".format(holdAngleDeg)}°")
    }

    fun startHeadingLock(robotId: String, targetAngleDeg: Float = 0f) {
        _robots.value[robotId] ?: return
        val tapState = _tapTargetState.value
        if (tapState?.active == true) {
            _tapTargetState.value = tapState.copy(active = false)
            if (tapState.robotId != robotId) {
                sendCommand(tapState.robotId, "A")
            }
        }
        sendCommand(robotId, "A")
        _headingLockState.value = HeadingLockState(
            robotId = robotId,
            targetAngleDeg = targetAngleDeg,
            lastCommandTimeMs = 0L
        )
        log("Heading lock started: Robot $robotId → ${"%.1f".format(targetAngleDeg)}°")
    }

    /**
     * Called every camera frame with the robot's observed position.
     *
     * KEY DESIGN: Monitoring happens EVERY FRAME (~30fps = ~33ms).
     * Commands are only sent/re-sent with a cooldown.
     * ABORT is sent INSTANTLY when the robot reaches tolerance.
     * This means the robot is caught within 1-2 frames of passing through the target.
     */
    fun updateTapTarget(robotId: String, curX: Float, curY: Float, curAngle: Float) {
        val state = _tapTargetState.value
        if (state == null || !state.active || state.robotId != robotId) return

        val now = System.currentTimeMillis()
        val dx = state.targetXmm - curX
        val dy = state.targetYmm - curY
        val distMm = kotlin.math.sqrt(dx * dx + dy * dy)

        val normalizedCurAngle = normalizeAngleToReference(curAngle, state.holdAngleDeg)
        var angleErr = state.holdAngleDeg - normalizedCurAngle
        while (angleErr > 180f) angleErr -= 360f
        while (angleErr < -180f) angleErr += 360f
        val absAngleErr = kotlin.math.abs(angleErr)

        // --- Arrived at position? (heading maintained by firmware during move) ---
        if (distMm <= state.arrivalToleranceMm) {
            sendCommand(robotId, "A")
            _tapTargetState.value = state.copy(
                active = false,
                lastCommandTimeMs = now
            )
            log("TT: reached (${"%.0f".format(state.targetXmm)},${"%.0f".format(state.targetYmm)}) err=${"%.0f".format(distMm)}mm angle=${"%.1f".format(angleErr)}°")
            return
        }

        // --- Large heading error → stop and rotate before moving ---
        if (absAngleErr > TT_HEADING_CORRECTION_THRESHOLD_DEG) {
            val cooldownMs = headingLockCooldownMs(absAngleErr)
            if (now - state.lastCommandTimeMs < cooldownMs) return

            val speed = headingLockSpeed(absAngleErr)
            val rotDir = rotationDirectionFromError(angleErr)
            val burstMs = headingLockBurstMs(absAngleErr)
            sendFreshCommand(robotId, "O:$rotDir:$burstMs:$speed:live")
            _tapTargetState.value = state.copy(lastCommandTimeMs = now)
            log("TT: heading err=${"%.1f".format(angleErr)}° > ${TT_HEADING_CORRECTION_THRESHOLD_DEG}° → rotate $rotDir [$speed/${burstMs}ms]")
            return
        }

        // --- Move toward target (firmware angle P-controller handles small drifts) ---
        if (now - state.lastCommandTimeMs < MOVE_COMMAND_COOLDOWN_MS) return

        sendFreshCommand(
            robotId,
            "W:${"%.1f".format(state.targetXmm)}:${"%.1f".format(state.targetYmm)}:slow:live"
        )
        _tapTargetState.value = state.copy(lastCommandTimeMs = now)
        log("TT: waypoint ${"%.0f".format(distMm)}mm heading=${"%.1f".format(absAngleErr)}° to (${"%.0f".format(state.targetXmm)},${"%.0f".format(state.targetYmm)})")
    }

    fun updateHeadingLock(robotId: String, curAngle: Float) {
        val state = _headingLockState.value
        if (state == null || !state.active || state.robotId != robotId) return

        val now = System.currentTimeMillis()
        val normalizedCurAngle = normalizeAngleToReference(curAngle, state.targetAngleDeg)
        var angleErr = state.targetAngleDeg - normalizedCurAngle
        while (angleErr > 180f) angleErr -= 360f
        while (angleErr < -180f) angleErr += 360f

        val absErr = kotlin.math.abs(angleErr)
        if (absErr <= state.angleToleranceDeg) {
            val nextStableFrameCount = state.stableFrameCount + 1
            if (state.stableFrameCount == 0) {
                sendCommand(robotId, "A")
            }
            if (nextStableFrameCount >= HEADING_LOCK_CONFIRM_FRAMES) {
                _headingLockState.value = state.copy(
                    active = false,
                    stableFrameCount = nextStableFrameCount,
                    lastCommandTimeMs = now
                )
                log("HL: locked ${"%.1f".format(curAngle)}° → ${"%.1f".format(state.targetAngleDeg)}° (err=${"%.1f".format(angleErr)}°)")
            } else {
                _headingLockState.value = state.copy(
                    stableFrameCount = nextStableFrameCount,
                    lastCommandTimeMs = now
                )
            }
            return
        }

        val activeState = if (state.stableFrameCount != 0) {
            state.copy(stableFrameCount = 0, lastCommandTimeMs = state.lastCommandTimeMs).also {
                _headingLockState.value = it
            }
        } else {
            state
        }

        val cooldownMs = headingLockCooldownMs(absErr)
        if (now - activeState.lastCommandTimeMs < cooldownMs) return

        val speed = headingLockSpeed(absErr)
        val rotDir = rotationDirectionFromError(angleErr)
        val burstMs = headingLockBurstMs(absErr)
        sendFreshCommand(robotId, "O:$rotDir:$burstMs:$speed:live")
        _headingLockState.value = activeState.copy(lastCommandTimeMs = now)
        log("HL: rotate burst $rotDir err=${"%.1f".format(angleErr)}° [$speed/${burstMs}ms/${cooldownMs}ms]")
    }

    /**
     * Progressive refinement: given the current angle error, return the
     * appropriate speed and the tolerance for that stage.
     *
     * Stages:
     *   >30°  → fast,  tolerate ±10°
     *   >10°  → normal, tolerate ±5°
     *   >2°   → slow,   tolerate ±2° (final)
     *   ≤2°   → done
     */
    private fun refinementStage(absErrorDeg: Float, finalTolerance: Float): Pair<String, Float> {
        return when {
            absErrorDeg > 30f -> "fast"   to 10f
            absErrorDeg > 10f -> "normal" to 5f
            else              -> "slow"   to finalTolerance
        }
    }

    private fun headingLockBurstMs(absErrorDeg: Float): Long {
        return when {
            absErrorDeg > 45f -> 95L
            absErrorDeg > 20f -> 85L
            absErrorDeg > 10f -> 80L
            absErrorDeg > 4f -> 75L
            absErrorDeg > 3f -> 60L
            else -> 50L
        }
    }

    private fun headingLockCooldownMs(absErrorDeg: Float): Long {
        return when {
            absErrorDeg > 45f -> 150L
            absErrorDeg > 20f -> 175L
            absErrorDeg > 10f -> 185L
            absErrorDeg > 4f -> 195L
            absErrorDeg > 3f -> 225L
            else -> 255L
        }
    }

    private fun headingLockSpeed(absErrorDeg: Float): String {
        return when {
            absErrorDeg > 45f -> "fast"
            absErrorDeg > 10f -> "normal"
            else -> "slow"
        }
    }

    /**
     * Convert a bearing angle (degrees, 0°=+X, 90°=+Y) to the closest
     * cardinal direction label for the protocol.
     */
    private fun angleToDirection(angleDeg: Float): String {
        var a = angleDeg % 360f
        if (a < 0f) a += 360f
        return when {
            a < 45f || a >= 315f -> "R"  // ~0°   = +X = RIGHT
            a < 135f             -> "U"  // ~90°  = +Y = UP
            a < 225f             -> "L"  // ~180° = -X = LEFT
            else                 -> "D"  // ~270° = -Y = DOWN
        }
    }

    private fun rotationDirectionFromError(angleErrDeg: Float): String {
        return if (angleErrDeg >= 0f) "ccw" else "cw"
    }

    /**
     * Normalize an angle to be within ±180° of a reference angle.
     * This prevents 0/360 oscillation when the target is near the wraparound point.
     * Example: ref=0°, angle=359° → returns -1° (which is 1° away from 0°)
     */
    private fun normalizeAngleToReference(angle: Float, reference: Float): Float {
        var diff = angle - reference
        while (diff > 180f) diff -= 360f
        while (diff < -180f) diff += 360f
        return reference + diff
    }

    fun cancelTapTarget() {
        val state = _tapTargetState.value ?: return
        _tapTargetState.value = state.copy(active = false)
        sendCommand(state.robotId, "A")
        log("Tap target cancelled")
    }

    fun cancelHeadingLock() {
        val state = _headingLockState.value ?: return
        _headingLockState.value = state.copy(active = false)
        sendCommand(state.robotId, "A")
        log("Heading lock cancelled")
    }

    val tapTargetState: StateFlow<TapTargetState?>
        get() = _tapTargetState.asStateFlow()

    val headingLockState: StateFlow<HeadingLockState?>
        get() = _headingLockState.asStateFlow()

    private val _tapTargetState = MutableStateFlow<TapTargetState?>(null)
    private val _headingLockState = MutableStateFlow<HeadingLockState?>(null)

    // ------------------------------------------------------------------
    // Low-level send
    // ------------------------------------------------------------------

    private fun sendRaw(address: InetAddress, port: Int, msg: String) {
        try {
            val data = msg.toByteArray(Charsets.US_ASCII)
            val pkt = DatagramPacket(data, data.size, address, port)
            socket?.send(pkt)
        } catch (e: Exception) {
            log("TX error: ${e.message}")
        }
    }

    fun isRobotConnected(robotId: String): Boolean {
        val robot = _robots.value[robotId] ?: return false
        return (System.currentTimeMillis() - robot.lastSeenMs) < 10_000 // 10s timeout
    }
}
