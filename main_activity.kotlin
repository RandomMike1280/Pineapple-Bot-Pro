package com.example.robot17promax

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.*
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.SocketException

// ============================================================================
// Data Models
// ============================================================================

data class RobotState(
    val id: String,
    var ip: String = "",
    var connected: Boolean = false,
    var posX: Float = 0f,
    var posY: Float = 0f,
    var queueDepth: Int = 0,
    var driftMm: Float = 0f,
    var rttMs: Long = 0,
    var lastUpdateTime: Long = 0
)

data class MoveCommand(
    val direction: String,   // "up", "down", "left", "right"
    val distanceMm: Int,
    val speed: String,       // "S" (slow), "N" (normal), "F" (fast)
    val correctionPolicy: String  // "L" (live), "D" (deferred), "N" (none)
)

// ============================================================================
// Main Activity
// ============================================================================

class MainActivity : ComponentActivity() {
    private val coordinator = RobotCoordinator()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            MaterialTheme {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = MaterialTheme.colorScheme.background
                ) {
                    CoordinatorUI(coordinator)
                }
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        coordinator.shutdown()
    }
}

// ============================================================================
// Robot Coordinator — manages both robots, routes messages
// ============================================================================

class RobotCoordinator {
    private val udpPort = 4210
    private var listeningSocket: DatagramSocket? = null
    private val coroutineScope = CoroutineScope(Dispatchers.IO)
    private var listenJob: Job? = null
    private var pingJob: Job? = null

    // --- Robot states (observable for Compose) ---
    var robotA = mutableStateOf(RobotState("A"))
    var robotB = mutableStateOf(RobotState("B"))
    var statusLog = mutableStateOf("")

    // --- Ping tracking ---
    private val pendingPings = mutableMapOf<String, MutableMap<Long, Long>>()  // robotId → {origTs → sentAtMs}

    init {
        pendingPings["A"] = mutableMapOf()
        pendingPings["B"] = mutableMapOf()
        startListening()
        startPingLoop()
    }

    // ---- Listening for incoming UDP from robots ----
    private fun startListening() {
        if (listenJob?.isActive == true) return
        listenJob = coroutineScope.launch {
            try {
                listeningSocket = DatagramSocket(udpPort)
                val buffer = ByteArray(512)
                while (isActive) {
                    val packet = DatagramPacket(buffer, buffer.size)
                    listeningSocket?.receive(packet)
                    val message = String(packet.data, 0, packet.length).trim()
                    val senderIp = packet.address.hostAddress ?: ""
                    handleIncoming(message, senderIp)
                }
            } catch (e: Exception) {
                if (e !is SocketException) {
                    log("Error: ${e.message}")
                }
            }
        }
    }

    private fun handleIncoming(message: String, senderIp: String) {
        val parts = message.split(":")
        if (parts.isEmpty()) return

        when (parts[0]) {
            "H" -> {
                // HELLO — H:<robot_id>
                // Also handle legacy "ESP32_HELLO" for backward compatibility
                val robotId = if (parts.size >= 2) parts[1] else "A"
                onRobotDiscovered(robotId, senderIp)
            }
            "R" -> {
                // REGISTER — R:<robot_id>:<capabilities>
                if (parts.size >= 2) {
                    val robotId = parts[1]
                    onRobotDiscovered(robotId, senderIp)
                    log("Robot $robotId registered (${parts.getOrElse(2) { "unknown" }})")
                }
            }
            "S" -> {
                // STATUS — S:<x>:<y>:<queue_len>:<drift>
                if (parts.size >= 5) {
                    val x = parts[1].toFloatOrNull() ?: 0f
                    val y = parts[2].toFloatOrNull() ?: 0f
                    val queueLen = parts[3].toIntOrNull() ?: 0
                    val drift = parts[4].toFloatOrNull() ?: 0f
                    updateRobotStatus(senderIp, x, y, queueLen, drift)
                }
            }
            "Q" -> {
                // PONG — Q:<orig_timestamp>
                if (parts.size >= 2) {
                    val origTs = parts[1].toLongOrNull() ?: return
                    onPongReceived(senderIp, origTs)
                }
            }
            else -> {
                // Legacy ESP32_HELLO support
                if (message.startsWith("ESP32_HELLO")) {
                    onRobotDiscovered("A", senderIp)
                }
            }
        }
    }

    private fun onRobotDiscovered(robotId: String, ip: String) {
        when (robotId) {
            "A" -> {
                val state = robotA.value
                if (!state.connected || state.ip != ip) {
                    robotA.value = state.copy(ip = ip, connected = true)
                    log("Robot A connected at $ip")
                }
            }
            "B" -> {
                val state = robotB.value
                if (!state.connected || state.ip != ip) {
                    robotB.value = state.copy(ip = ip, connected = true)
                    log("Robot B connected at $ip")
                }
            }
        }
    }

    private fun updateRobotStatus(senderIp: String, x: Float, y: Float, queueLen: Int, drift: Float) {
        val now = System.currentTimeMillis()
        if (robotA.value.ip == senderIp) {
            robotA.value = robotA.value.copy(
                posX = x, posY = y, queueDepth = queueLen,
                driftMm = drift, lastUpdateTime = now
            )
        } else if (robotB.value.ip == senderIp) {
            robotB.value = robotB.value.copy(
                posX = x, posY = y, queueDepth = queueLen,
                driftMm = drift, lastUpdateTime = now
            )
        }
    }

    // ---- Ping/RTT measurement ----
    private fun startPingLoop() {
        pingJob = coroutineScope.launch {
            while (isActive) {
                delay(500)
                sendPingTo("A")
                sendPingTo("B")
            }
        }
    }

    private fun sendPingTo(robotId: String) {
        val state = if (robotId == "A") robotA.value else robotB.value
        if (!state.connected || state.ip.isBlank()) return

        val now = System.currentTimeMillis()
        val message = "P:$now"
        pendingPings[robotId]?.put(now, now)
        sendRaw(state.ip, message)
    }

    private fun onPongReceived(senderIp: String, origTimestamp: Long) {
        val now = System.currentTimeMillis()
        val rtt = now - origTimestamp

        if (robotA.value.ip == senderIp) {
            robotA.value = robotA.value.copy(rttMs = rtt)
            pendingPings["A"]?.remove(origTimestamp)
        } else if (robotB.value.ip == senderIp) {
            robotB.value = robotB.value.copy(rttMs = rtt)
            pendingPings["B"]?.remove(origTimestamp)
        }
    }

    // ---- Command API ----

    fun sendMoveCommand(robotId: String, cmd: MoveCommand) {
        val state = if (robotId == "A") robotA.value else robotB.value
        if (!state.connected) {
            log("Cannot send to Robot $robotId — not connected")
            return
        }

        // Collision avoidance check
        val other = if (robotId == "A") robotB.value else robotA.value
        if (other.connected && checkCollisionRisk(state, other, cmd)) {
            log("⚠ Collision risk detected! Command held.")
            return
        }

        val dirCode = when (cmd.direction) {
            "up" -> "U"; "down" -> "D"; "left" -> "L"; "right" -> "R"; else -> "U"
        }
        val message = "M:$dirCode:${cmd.distanceMm}:${cmd.speed}:${cmd.correctionPolicy}"
        sendRaw(state.ip, message)
        log("→ Robot $robotId: ${cmd.direction} ${cmd.distanceMm}mm [${cmd.speed}]")
    }

    fun sendMoveSequence(robotId: String, commands: List<MoveCommand>) {
        for (cmd in commands) {
            sendMoveCommand(robotId, cmd)
        }
        log("→ Robot $robotId: queued ${commands.size} moves")
    }

    fun sendCameraObservation(robotId: String, x: Float, y: Float) {
        val state = if (robotId == "A") robotA.value else robotB.value
        if (!state.connected) return

        val timestamp = System.currentTimeMillis()
        val message = "C:$timestamp:$x:$y"
        sendRaw(state.ip, message)
    }

    fun sendAbort(robotId: String) {
        val state = if (robotId == "A") robotA.value else robotB.value
        if (!state.connected) return
        sendRaw(state.ip, "A")
        log("⛔ Robot $robotId: ABORT")
    }

    // ---- Collision avoidance (simple distance check) ----
    private fun checkCollisionRisk(mover: RobotState, other: RobotState, cmd: MoveCommand): Boolean {
        // Project the mover's future position
        val dx = when (cmd.direction) { "right" -> cmd.distanceMm.toFloat(); "left" -> -cmd.distanceMm.toFloat(); else -> 0f }
        val dy = when (cmd.direction) { "up" -> cmd.distanceMm.toFloat(); "down" -> -cmd.distanceMm.toFloat(); else -> 0f }
        val futureX = mover.posX + dx
        val futureY = mover.posY + dy

        // Check if future position is within safety margin of other robot
        val safetyMargin = 150f // mm — adjust based on robot size
        val distSq = (futureX - other.posX) * (futureX - other.posX) +
                     (futureY - other.posY) * (futureY - other.posY)
        return distSq < safetyMargin * safetyMargin
    }

    // ---- Raw UDP send ----
    private fun sendRaw(ip: String, message: String) {
        if (ip.isBlank()) return
        coroutineScope.launch {
            try {
                val address = InetAddress.getByName(ip)
                val bytes = message.toByteArray()
                val packet = DatagramPacket(bytes, bytes.size, address, udpPort)
                val socket = DatagramSocket()
                socket.send(packet)
                socket.close()
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
    }

    private fun log(msg: String) {
        val existing = statusLog.value
        val lines = existing.split("\n")
        // Keep last 20 log lines
        val trimmed = if (lines.size > 20) lines.takeLast(20).joinToString("\n") else existing
        statusLog.value = "$trimmed\n$msg"
    }

    fun shutdown() {
        listenJob?.cancel()
        pingJob?.cancel()
        listeningSocket?.close()
        listeningSocket = null
        coroutineScope.cancel()
    }
}

// ============================================================================
// Composable UI
// ============================================================================

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun CoordinatorUI(coordinator: RobotCoordinator) {
    val robotA = coordinator.robotA.value
    val robotB = coordinator.robotB.value
    var selectedRobot by remember { mutableStateOf("A") }
    var distanceMm by remember { mutableIntStateOf(200) }
    var speedLevel by remember { mutableStateOf("N") }
    var correctionPolicy by remember { mutableStateOf("L") }
    val scrollState = rememberScrollState()

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(16.dp)
            .verticalScroll(scrollState),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        // ---- Title ----
        Text(
            text = "Robot Coordinator",
            style = MaterialTheme.typography.headlineMedium,
            fontWeight = FontWeight.Bold
        )
        Spacer(modifier = Modifier.height(12.dp))

        // ---- Robot Status Cards ----
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            RobotStatusCard(
                robot = robotA,
                isSelected = selectedRobot == "A",
                onSelect = { selectedRobot = "A" },
                modifier = Modifier.weight(1f)
            )
            RobotStatusCard(
                robot = robotB,
                isSelected = selectedRobot == "B",
                onSelect = { selectedRobot = "B" },
                modifier = Modifier.weight(1f)
            )
        }

        Spacer(modifier = Modifier.height(16.dp))

        // ---- D-Pad Controls ----
        Text("Move Robot $selectedRobot", style = MaterialTheme.typography.titleMedium)
        Spacer(modifier = Modifier.height(8.dp))

        DirectionPad(
            onDirection = { direction ->
                coordinator.sendMoveCommand(
                    selectedRobot,
                    MoveCommand(direction, distanceMm, speedLevel, correctionPolicy)
                )
            }
        )

        Spacer(modifier = Modifier.height(12.dp))

        // ---- Distance Slider ----
        Text("Distance: ${distanceMm}mm")
        Slider(
            value = distanceMm.toFloat(),
            onValueChange = { distanceMm = it.toInt() },
            valueRange = 50f..2000f,
            steps = 38,
            modifier = Modifier.padding(horizontal = 24.dp)
        )

        Spacer(modifier = Modifier.height(8.dp))

        // ---- Speed Selection ----
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            SpeedButton("Slow", "S", speedLevel) { speedLevel = "S" }
            SpeedButton("Normal", "N", speedLevel) { speedLevel = "N" }
            SpeedButton("Fast", "F", speedLevel) { speedLevel = "F" }
        }

        Spacer(modifier = Modifier.height(8.dp))

        // ---- Correction Policy Selection ----
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            PolicyButton("Live", "L", correctionPolicy) { correctionPolicy = "L" }
            PolicyButton("Deferred", "D", correctionPolicy) { correctionPolicy = "D" }
            PolicyButton("None", "N", correctionPolicy) { correctionPolicy = "N" }
        }

        Spacer(modifier = Modifier.height(12.dp))

        // ---- Emergency Stop ----
        Button(
            onClick = {
                coordinator.sendAbort("A")
                coordinator.sendAbort("B")
            },
            colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error),
            modifier = Modifier.fillMaxWidth()
        ) {
            Text("⛔ EMERGENCY STOP ALL", fontWeight = FontWeight.Bold, fontSize = 16.sp)
        }

        Spacer(modifier = Modifier.height(12.dp))

        // ---- Status Log ----
        Text("Log", style = MaterialTheme.typography.titleSmall)
        Surface(
            modifier = Modifier
                .fillMaxWidth()
                .height(160.dp),
            shape = RoundedCornerShape(8.dp),
            color = MaterialTheme.colorScheme.surfaceVariant
        ) {
            Text(
                text = coordinator.statusLog.value,
                modifier = Modifier.padding(8.dp),
                fontSize = 11.sp,
                lineHeight = 14.sp
            )
        }
    }
}

// ---- Robot Status Card ----
@Composable
fun RobotStatusCard(
    robot: RobotState,
    isSelected: Boolean,
    onSelect: () -> Unit,
    modifier: Modifier = Modifier
) {
    val borderColor = if (isSelected) MaterialTheme.colorScheme.primary else Color.Transparent
    val statusColor = if (robot.connected) Color(0xFF4CAF50) else Color(0xFFFF5722)

    Card(
        modifier = modifier
            .border(2.dp, borderColor, RoundedCornerShape(12.dp))
            .clickable { onSelect() },
        shape = RoundedCornerShape(12.dp)
    ) {
        Column(modifier = Modifier.padding(12.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Box(
                    modifier = Modifier
                        .size(10.dp)
                        .clip(CircleShape)
                        .background(statusColor)
                )
                Spacer(modifier = Modifier.width(6.dp))
                Text(
                    "Robot ${robot.id}",
                    fontWeight = FontWeight.Bold,
                    fontSize = 16.sp
                )
            }
            Spacer(modifier = Modifier.height(4.dp))
            if (robot.connected) {
                Text("Pos: (${robot.posX.toInt()}, ${robot.posY.toInt()}) mm", fontSize = 12.sp)
                Text("Queue: ${robot.queueDepth} | Drift: ${"%.1f".format(robot.driftMm)}mm", fontSize = 12.sp)
                Text("RTT: ${robot.rttMs}ms | IP: ${robot.ip}", fontSize = 11.sp)
            } else {
                Text("Waiting for connection...", fontSize = 12.sp, color = Color.Gray)
            }
        }
    }
}

// ---- Direction Pad ----
@Composable
fun DirectionPad(onDirection: (String) -> Unit) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        DPadButton(Icons.Default.KeyboardArrowUp, "up", onDirection)
        Row(horizontalArrangement = Arrangement.spacedBy(24.dp)) {
            DPadButton(Icons.Default.KeyboardArrowLeft, "left", onDirection)
            DPadButton(Icons.Default.KeyboardArrowDown, "down", onDirection)
            DPadButton(Icons.Default.KeyboardArrowRight, "right", onDirection)
        }
    }
}

@Composable
fun DPadButton(icon: ImageVector, direction: String, onClick: (String) -> Unit) {
    FilledTonalButton(
        onClick = { onClick(direction) },
        modifier = Modifier.size(64.dp),
        shape = RoundedCornerShape(12.dp)
    ) {
        Icon(icon, contentDescription = direction, modifier = Modifier.size(32.dp))
    }
}

// ---- Speed Toggle Button ----
@Composable
fun RowScope.SpeedButton(label: String, value: String, selected: String, onClick: () -> Unit) {
    val isSelected = value == selected
    if (isSelected) {
        Button(onClick = onClick, modifier = Modifier.weight(1f)) { Text(label) }
    } else {
        OutlinedButton(onClick = onClick, modifier = Modifier.weight(1f)) { Text(label) }
    }
}

// ---- Correction Policy Toggle Button ----
@Composable
fun RowScope.PolicyButton(label: String, value: String, selected: String, onClick: () -> Unit) {
    val isSelected = value == selected
    if (isSelected) {
        Button(onClick = onClick, modifier = Modifier.weight(1f)) { Text(label, fontSize = 13.sp) }
    } else {
        OutlinedButton(onClick = onClick, modifier = Modifier.weight(1f)) { Text(label, fontSize = 13.sp) }
    }
}