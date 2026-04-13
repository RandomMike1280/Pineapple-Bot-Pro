package com.example.robot17promax

import android.os.Bundle
import android.util.Size
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.camera.core.*
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import kotlinx.coroutines.*
import org.opencv.android.OpenCVLoader
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.opencv.objdetect.ArucoDetector
import org.opencv.objdetect.DetectorParameters
import org.opencv.objdetect.Objdetect
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.SocketException
import kotlin.math.sqrt

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

data class WaypointCommand(
    val x: Float,
    val y: Float,
    val speed: String,       // "S" (slow), "N" (normal), "F" (fast)
    val correctionPolicy: String  // "L" (live), "D" (deferred), "N" (none)
)

data class FieldConfig(
    val widthMm: Float = 2000f,
    val heightMm: Float = 2000f,
    val isLocked: Boolean = false,
    // [TL, TR, BR, BL] normalized coordinates (0.0 to 1.0) of the camera preview view
    val corners: List<Offset> = listOf(
        Offset(0.1f, 0.1f), Offset(0.9f, 0.1f),
        Offset(0.9f, 0.9f), Offset(0.1f, 0.9f)
    )
)

// ============================================================================
// Main Activity
// ============================================================================

class MainActivity : ComponentActivity() {
    private val coordinator = RobotCoordinator()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            println("OpenCV computation failed to initialize.")
        }

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
// Robot Coordinator
// ============================================================================

class RobotCoordinator {
    private val udpPort = 4210
    private var listeningSocket: DatagramSocket? = null
    private val coroutineScope = CoroutineScope(Dispatchers.IO)
    private var listenJob: Job? = null
    private var pingJob: Job? = null

    var robotA = mutableStateOf(RobotState("A"))
    var robotB = mutableStateOf(RobotState("B"))
    var statusLog = mutableStateOf("")

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
                val id = if (parts.size >= 2) parts[1] else "A"
                val ip = if (parts.size >= 3) parts[2] else senderIp
                onRobotDiscovered(id, ip.ifBlank { senderIp })
            }
            "R" -> {
                val id = if (parts.size >= 2) parts[1] else "A"
                // R:<id>:<caps>:<ip> — IP is the last field
                val ip = if (parts.size >= 4) parts[3] else senderIp
                onRobotDiscovered(id, ip.ifBlank { senderIp })
            }
            "S" -> {
                if (parts.size >= 5) {
                    updateRobotStatus(
                        senderIp, parts[1].toFloatOrNull() ?: 0f, parts[2].toFloatOrNull() ?: 0f,
                        parts[3].toIntOrNull() ?: 0, parts[4].toFloatOrNull() ?: 0f
                    )
                }
            }
            "Q" -> {
                if (parts.size >= 2) parts[1].toLongOrNull()?.let { onPongReceived(senderIp, it) }
            }
            else -> {
                if (message.startsWith("ESP32_HELLO")) onRobotDiscovered("A", senderIp)
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
            robotA.value = robotA.value.copy(posX = x, posY = y, queueDepth = queueLen, driftMm = drift, lastUpdateTime = now)
        } else if (robotB.value.ip == senderIp) {
            robotB.value = robotB.value.copy(posX = x, posY = y, queueDepth = queueLen, driftMm = drift, lastUpdateTime = now)
        }
    }

    // ---- Ping tracking (omitted exact logic for brevity, keeping same style as original) ----
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
        pendingPings[robotId]?.put(now, now)
        sendRaw(state.ip, "P:$now")
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

    fun sendWaypointCommand(robotId: String, cmd: WaypointCommand) {
        val state = if (robotId == "A") robotA.value else robotB.value
        if (!state.connected) {
            log("Cannot send to Robot $robotId — not connected")
            return
        }

        // Collision avoidance check
        val other = if (robotId == "A") robotB.value else robotA.value
        if (other.connected && checkCollisionRisk(cmd.x, cmd.y, other)) {
            log("⚠ Collision risk detected! Waypoint held.")
            return
        }

        val message = "W:${cmd.x}:${cmd.y}:${cmd.speed}:${cmd.correctionPolicy}"
        sendRaw(state.ip, message)
        log("→ Robot $robotId: move to (${cmd.x.toInt()}, ${cmd.y.toInt()}) [${cmd.speed}]")
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

    private fun checkCollisionRisk(targetX: Float, targetY: Float, other: RobotState): Boolean {
        val safetyMargin = 300f
        val distSq = (targetX - other.posX) * (targetX - other.posX) + (targetY - other.posY) * (targetY - other.posY)
        return distSq < safetyMargin * safetyMargin
    }

    private fun sendRaw(ip: String, message: String) {
        if (ip.isBlank()) return
        coroutineScope.launch {
            try {
                // Parse numeric IP directly to avoid DNS resolution failures
                // on hotspots that lack a DNS server
                val address = parseNumericIp(ip) ?: InetAddress.getByName(ip)
                val bytes = message.toByteArray()
                val packet = DatagramPacket(bytes, bytes.size, address, udpPort)
                val socket = DatagramSocket()
                socket.send(packet)
                socket.close()
            } catch (e: Exception) {
                log("SYS Error TX: ${e.message}")
            }
        }
    }

    private fun parseNumericIp(ip: String): InetAddress? {
        val parts = ip.split(".")
        if (parts.size != 4) return null
        val bytes = ByteArray(4)
        for (i in 0..3) {
            val v = parts[i].toIntOrNull() ?: return null
            if (v < 0 || v > 255) return null
            bytes[i] = v.toByte()
        }
        return InetAddress.getByAddress(bytes)
    }

    private fun log(msg: String) {
        val existing = statusLog.value
        val lines = existing.split("\n")
        val trimmed = if (lines.size > 10) lines.takeLast(10).joinToString("\n") else existing
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
// OpenCV Vision Tracker
// ============================================================================

class ArUcoTracker(
    private val coordinator: RobotCoordinator,
    private val fieldConfig: FieldConfig
) : ImageAnalysis.Analyzer {

    // Aruco ID Map: 0 = Robot A, 1 = Robot B
    private val dict = Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50)
    private val params = DetectorParameters()
    private val detector = ArucoDetector(dict, params)

    private var perspectiveTransform: Mat? = null

    init {
        updateHomography()
    }

    fun updateHomography() {
        // Source points: The normalized [0..1] coordinates * internal resolution of image
        // To simplify, we calculate homography per-frame based on the image size, 
        // preventing resolution mismatch issues.
    }

    @androidx.annotation.OptIn(androidx.camera.core.ExperimentalGetImage::class)
    override fun analyze(imageProxy: ImageProxy) {
        if (!fieldConfig.isLocked) {
            imageProxy.close()
            return
        }

        val image = imageProxy.image ?: return
        
        // Convert YUV to OpenCV Mat (omitted full YUV2RGB for brevity, using gray directly)
        val yBuffer = image.planes[0].buffer
        val ySize = yBuffer.remaining()
        val yArray = ByteArray(ySize)
        yBuffer.get(yArray)
        
        val matGray = Mat(image.height, image.width, CvType.CV_8UC1)
        matGray.put(0, 0, yArray)

        // 1. Create projection matrix based on locked screen coordinates 
        // image coordinate mapping:
        val srcMat = Mat(4, 1, CvType.CV_32FC2)
        val dstMat = Mat(4, 1, CvType.CV_32FC2)
        
        // Map 4 corner pins from normalized coordinates -> image pixel coordinates
        // Assuming the UI layout matches the camera aspect ratio
        srcMat.put(0, 0, 
            fieldConfig.corners[0].x * image.width.toDouble(), fieldConfig.corners[0].y * image.height.toDouble(),
            fieldConfig.corners[1].x * image.width.toDouble(), fieldConfig.corners[1].y * image.height.toDouble(),
            fieldConfig.corners[2].x * image.width.toDouble(), fieldConfig.corners[2].y * image.height.toDouble(),
            fieldConfig.corners[3].x * image.width.toDouble(), fieldConfig.corners[3].y * image.height.toDouble()
        )

        // Map them to physical field geometry (mm)
        // TL=(0,H), TR=(W,H), BR=(W,0), BL=(0,0)   (assuming Y is up)
        dstMat.put(0, 0,
            0.0, fieldConfig.heightMm.toDouble(),
            fieldConfig.widthMm.toDouble(), fieldConfig.heightMm.toDouble(),
            fieldConfig.widthMm.toDouble(), 0.0,
            0.0, 0.0
        )

        perspectiveTransform = Imgproc.getPerspectiveTransform(srcMat, dstMat)

        // 2. Detect Aruco Markers
        val corners = ArrayList<Mat>()
        val ids = Mat()
        detector.detectMarkers(matGray, corners, ids)

        // 3. Transform detected pixels to physical mm
        if (ids.rows() > 0) {
            val idsArray = IntArray(ids.total().toInt())
            ids.get(0, 0, idsArray)

            for (i in idsArray.indices) {
                val markerId = idsArray[i]
                val cornerMat = corners[i]
                
                // Calculate centroid of the 4 corners of the marker in pixel space
                var cx = 0.0
                var cy = 0.0
                for (j in 0..3) {
                    val px = cornerMat.get(0, j)
                    cx += px[0]
                    cy += px[1]
                }
                cx /= 4.0
                cy /= 4.0

                // Apply Perspective Transform
                val ptSrc = Mat(1, 1, CvType.CV_32FC2)
                ptSrc.put(0, 0, cx, cy)
                val ptDst = Mat()
                
                if (perspectiveTransform != null) {
                    org.opencv.core.Core.perspectiveTransform(ptSrc, ptDst, perspectiveTransform)
                    val realX = ptDst.get(0, 0)[0].toFloat()
                    val realY = ptDst.get(0, 0)[1].toFloat()

                    // Dispatch to coordinator
                    val robotId = if (markerId == 0) "A" else "B"
                    coordinator.sendCameraObservation(robotId, realX, realY)
                }
            }
        }

        matGray.release()
        imageProxy.close()
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
    var speedLevel by remember { mutableStateOf("N") }
    var correctionPolicy by remember { mutableStateOf("L") }
    val scrollState = rememberScrollState()

    // Field definition state
    var fieldConfig by remember { mutableStateOf(FieldConfig()) }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .verticalScroll(scrollState),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        // ---- Camera Tracker Overlay ----
        Box(modifier = Modifier
            .fillMaxWidth()
            .height(350.dp)
            .background(Color.Black)) {
            
            // 1. CameraX Preview
            CameraTrackerView(coordinator, fieldConfig)

            // 2. Interactive Boundary Overlay
            if (!fieldConfig.isLocked) {
                BoundaryOverlay(
                    corners = fieldConfig.corners,
                    onCornersChanged = { newCorners -> 
                        fieldConfig = fieldConfig.copy(corners = newCorners) 
                    }
                )
            }
        }

        // ---- Settings Strip ----
        Row(
            modifier = Modifier.fillMaxWidth().padding(8.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Button(
                onClick = { fieldConfig = fieldConfig.copy(isLocked = !fieldConfig.isLocked) },
                colors = ButtonDefaults.buttonColors(
                    containerColor = if (fieldConfig.isLocked) Color(0xFF4CAF50) else Color(0xFFFF9800)
                )
            ) {
                Text(if (fieldConfig.isLocked) "Tracking Locked" else "Lock Map Boundaries", fontWeight = FontWeight.Bold)
            }
        }

        Column(modifier = Modifier.padding(16.dp)) {
            // ---- Robot Status Cards ----
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                RobotStatusCard(robotA, selectedRobot == "A", { selectedRobot = "A" }, Modifier.weight(1f))
                RobotStatusCard(robotB, selectedRobot == "B", { selectedRobot = "B" }, Modifier.weight(1f))
            }
            Spacer(modifier = Modifier.height(16.dp))

            // ---- Waypoint Control ----
            Text("Send Manual Waypoint ($selectedRobot)", style = MaterialTheme.typography.titleMedium)
            var targetX by remember { mutableStateOf("1000") }
            var targetY by remember { mutableStateOf("1000") }

            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedTextField(value = targetX, onValueChange = { targetX = it }, label = { Text("X (mm)") }, modifier = Modifier.weight(1f))
                OutlinedTextField(value = targetY, onValueChange = { targetY = it }, label = { Text("Y (mm)") }, modifier = Modifier.weight(1f))
                Button(onClick = {
                    coordinator.sendWaypointCommand(
                        selectedRobot, WaypointCommand(targetX.toFloatOrNull() ?: 0f, targetY.toFloatOrNull() ?: 0f, speedLevel, correctionPolicy)
                    )
                }, modifier = Modifier.align(Alignment.CenterVertically)) {
                    Text("GO")
                }
            }

            Spacer(modifier = Modifier.height(12.dp))

            // ---- Options ----
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                SpeedButton("Slow", "S", speedLevel) { speedLevel = "S" }
                SpeedButton("Normal", "N", speedLevel) { speedLevel = "N" }
                SpeedButton("Fast", "F", speedLevel) { speedLevel = "F" }
            }
        }
    }
}

// ---- Camera View + Vision Setup ----
@Composable
fun CameraTrackerView(coordinator: RobotCoordinator, fieldConfig: FieldConfig) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current

    val cameraProviderFuture = remember { ProcessCameraProvider.getInstance(context) }
    val analyzer = remember(fieldConfig.isLocked) { ArUcoTracker(coordinator, fieldConfig) }

    AndroidView(
        factory = { ctx ->
            val previewView = PreviewView(ctx)
            val cameraProvider = cameraProviderFuture.get()

            val preview = Preview.Builder().build().also {
                it.setSurfaceProvider(previewView.surfaceProvider)
            }

            val imageAnalysis = ImageAnalysis.Builder()
                .setTargetResolution(Size(1280, 720))
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build()
                .also {
                    it.setAnalyzer(ContextCompat.getMainExecutor(ctx), analyzer)
                }

            val cameraSelector = CameraSelector.DEFAULT_BACK_CAMERA

            try {
                cameraProvider.unbindAll()
                cameraProvider.bindToLifecycle(lifecycleOwner, cameraSelector, preview, imageAnalysis)
            } catch (e: Exception) {
                e.printStackTrace()
            }
            previewView
        },
        modifier = Modifier.fillMaxSize()
    )
}

// ---- Draggable Boundary Overlay ----
@Composable
fun BoundaryOverlay(corners: List<Offset>, onCornersChanged: (List<Offset>) -> Unit) {
    Canvas(modifier = Modifier.fillMaxSize().pointerInput(Unit) {
        detectDragGestures { change, dragAmount ->
            change.consume()
            val canvasWidth = size.width
            val canvasHeight = size.height

            // Find nearest corner to the drag
            val currentTouch = change.position
            var nearestIdx = 0
            var minDist = Float.MAX_VALUE

            corners.forEachIndexed { i, corner ->
                val px = corner.x * canvasWidth
                val py = corner.y * canvasHeight
                val d = sqrt((currentTouch.x - px) * (currentTouch.x - px) + (currentTouch.y - py) * (currentTouch.y - py))
                if (d < minDist) {
                    minDist = d
                    nearestIdx = i
                }
            }

            // Only drag if within a reasonable touch radius (e.g. 100 pixels)
            if (minDist < 100f) {
                val dxNorm = dragAmount.x / canvasWidth
                val dyNorm = dragAmount.y / canvasHeight
                
                val mutList = corners.toMutableList()
                mutList[nearestIdx] = Offset(
                    x = (mutList[nearestIdx].x + dxNorm).coerceIn(0f, 1f),
                    y = (mutList[nearestIdx].y + dyNorm).coerceIn(0f, 1f)
                )
                onCornersChanged(mutList)
            }
        }
    }) {
        val w = size.width
        val h = size.height

        val p0 = Offset(corners[0].x * w, corners[0].y * h)
        val p1 = Offset(corners[1].x * w, corners[1].y * h)
        val p2 = Offset(corners[2].x * w, corners[2].y * h)
        val p3 = Offset(corners[3].x * w, corners[3].y * h)

        val strokeColor = Color.Yellow
        val pinColor = Color.Red

        // Draw boundary box
        drawLine(strokeColor, p0, p1, 4f)
        drawLine(strokeColor, p1, p2, 4f)
        drawLine(strokeColor, p2, p3, 4f)
        drawLine(strokeColor, p3, p0, 4f)

        // Draw corner pins
        drawCircle(pinColor, 24f, p0)
        drawCircle(pinColor, 24f, p1)
        drawCircle(pinColor, 24f, p2)
        drawCircle(pinColor, 24f, p3)
    }
}

// ---- Legacy UI Elements ----
@Composable
fun RobotStatusCard(robot: RobotState, isSelected: Boolean, onSelect: () -> Unit, modifier: Modifier = Modifier) {
    val borderColor = if (isSelected) MaterialTheme.colorScheme.primary else Color.Transparent
    val statusColor = if (robot.connected) Color(0xFF4CAF50) else Color(0xFFFF5722)

    Card(modifier = modifier.border(2.dp, borderColor, RoundedCornerShape(12.dp)).clickable { onSelect() }, shape = RoundedCornerShape(12.dp)) {
        Column(modifier = Modifier.padding(12.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Box(modifier = Modifier.size(10.dp).clip(CircleShape).background(statusColor))
                Spacer(modifier = Modifier.width(6.dp))
                Text("Robot ${robot.id}", fontWeight = FontWeight.Bold)
            }
            if (robot.connected) {
                Text("Pos: (${robot.posX.toInt()}, ${robot.posY.toInt()})", fontSize = 12.sp)
                Text("RTT: ${robot.rttMs}ms", fontSize = 11.sp)
            }
        }
    }
}

@Composable
fun RowScope.SpeedButton(label: String, value: String, selected: String, onClick: () -> Unit) {
    val isSelected = value == selected
    if (isSelected) Button(onClick = onClick, modifier = Modifier.weight(1f)) { Text(label) }
    else OutlinedButton(onClick = onClick, modifier = Modifier.weight(1f)) { Text(label) }
}