package com.example.pineapplerobotpro

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.*
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import android.hardware.camera2.CameraCharacteristics
import androidx.compose.animation.*
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.layout.positionInParent
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.IntSize
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import org.opencv.android.OpenCVLoader
import org.opencv.android.Utils
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.opencv.objdetect.ArucoDetector
import org.opencv.objdetect.DetectorParameters
import org.opencv.objdetect.Objdetect
import kotlin.math.atan2
import kotlin.math.roundToInt
import kotlin.math.sqrt

// ============================================================================
// Data Models
// ============================================================================

// ============================================================================
// Marker ID → Robot ID mapping
// ============================================================================
// ArUco marker ID on each robot maps to its UDP robot ID.
// Adjust these if your markers use different IDs.
val MARKER_TO_ROBOT = mapOf(
    0 to "A",
    1 to "B"
)

// Field dimensions in mm (used to convert normalized coords to mm)
const val FIELD_WIDTH_MM = 1140f
const val FIELD_HEIGHT_MM = 1180f

data class DetectedMarker(
    val id: Int,
    val centerX: Float,   // Normalized relative to boundary (can be <0 or >1 if outside)
    val centerY: Float,
    val corners: List<Offset>
)

data class WarpResult(
    val bitmap: Bitmap,
    val markers: List<DetectedMarker>,
    val timestampMs: Long
)

private fun markerObservation(marker: DetectedMarker): Triple<Float, Float, Float> {
    val xMm = (marker.centerX) * FIELD_WIDTH_MM
    val yMm = (marker.centerY) * FIELD_HEIGHT_MM
    var angleDeg = 0f
    if (marker.corners.size == 4) {
        val c0 = marker.corners[0]
        val c1 = marker.corners[1]
        val topMidX = (c0.x + c1.x) / 2f
        val topMidY = (c0.y + c1.y) / 2f
        val dx = topMidX - marker.centerX
        val dy = topMidY - marker.centerY
        angleDeg = Math.toDegrees(atan2(-dy.toDouble(), dx.toDouble())).toFloat()
        if (angleDeg < 0f) angleDeg += 360f
    }
    return Triple(xMm, yMm, angleDeg)
}

// ============================================================================
// Trail Color Palette
// ============================================================================

val TRAIL_COLORS = listOf(
    Color(0xFF00FFFF), // Cyan
    Color(0xFFFF00FF), // Magenta
    Color(0xFFFFFF00), // Yellow
    Color(0xFF00FF7F), // Spring Green
    Color(0xFFFF6B6B), // Coral
    Color(0xFF6B9EFF), // Cornflower Blue
    Color(0xFFFFB347), // Orange
    Color(0xFF98FB98), // Pale Green
)

// ============================================================================
// Activity
// ============================================================================

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        if (!OpenCVLoader.initDebug()) {
            println("OpenCV failed to initialize.")
        }

        setContent {
            MaterialTheme {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = MaterialTheme.colorScheme.background
                ) {
                    CameraApp()
                }
            }
        }
    }
}

// ============================================================================
// Permission Screen
// ============================================================================

@Composable
fun CameraApp() {
    val context = LocalContext.current
    var hasCameraPermission by remember {
        mutableStateOf(
            ContextCompat.checkSelfPermission(
                context,
                Manifest.permission.CAMERA
            ) == PackageManager.PERMISSION_GRANTED
        )
    }

    val permissionLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestPermission(),
        onResult = { granted -> hasCameraPermission = granted }
    )

    LaunchedEffect(Unit) {
        if (!hasCameraPermission) {
            permissionLauncher.launch(Manifest.permission.CAMERA)
        }
    }

    if (hasCameraPermission) {
        PerspectiveWarpScreen()
    } else {
        Box(contentAlignment = Alignment.Center, modifier = Modifier.fillMaxSize()) {
            Text("Camera permission is required to use this app.")
        }
    }
}

// ============================================================================
// Top Control Bar
// ============================================================================

enum class ActiveControl { NONE, TRAIL_LENGTH, TRACKING_AREA }

@Composable
fun ControlBar(
    maxTrailLength: Int,
    onMaxTrailLengthChange: (Int) -> Unit,
    trackingPadding: Float,
    onTrackingPaddingChange: (Float) -> Unit
) {
    var activeControl by remember { mutableStateOf(ActiveControl.NONE) }

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .background(Color(0xFF16213E).copy(alpha = 0.92f))
            .padding(horizontal = 12.dp, vertical = 8.dp)
    ) {
        // Control chips row
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            FilterChip(
                selected = activeControl == ActiveControl.TRAIL_LENGTH,
                onClick = {
                    activeControl = if (activeControl == ActiveControl.TRAIL_LENGTH)
                        ActiveControl.NONE else ActiveControl.TRAIL_LENGTH
                },
                label = { Text("📏 Trail: $maxTrailLength", fontSize = 12.sp) },
                colors = FilterChipDefaults.filterChipColors(
                    selectedContainerColor = Color(0xFF4CAF50).copy(alpha = 0.3f),
                    selectedLabelColor = Color.White,
                    labelColor = Color.White.copy(alpha = 0.7f)
                )
            )

            val paddingPercent = (trackingPadding * 100).roundToInt()
            FilterChip(
                selected = activeControl == ActiveControl.TRACKING_AREA,
                onClick = {
                    activeControl = if (activeControl == ActiveControl.TRACKING_AREA)
                        ActiveControl.NONE else ActiveControl.TRACKING_AREA
                },
                label = { Text("🔍 Area: +${paddingPercent}%", fontSize = 12.sp) },
                colors = FilterChipDefaults.filterChipColors(
                    selectedContainerColor = Color(0xFF2196F3).copy(alpha = 0.3f),
                    selectedLabelColor = Color.White,
                    labelColor = Color.White.copy(alpha = 0.7f)
                )
            )
        }

        // Expandable slider area
        AnimatedVisibility(
            visible = activeControl != ActiveControl.NONE,
            enter = expandVertically() + fadeIn(),
            exit = shrinkVertically() + fadeOut()
        ) {
            when (activeControl) {
                ActiveControl.TRAIL_LENGTH -> {
                    Column(modifier = Modifier.padding(top = 4.dp)) {
                        Text(
                            "Max Trail Length: $maxTrailLength points",
                            color = Color.White.copy(alpha = 0.7f),
                            fontSize = 11.sp
                        )
                        Slider(
                            value = maxTrailLength.toFloat(),
                            onValueChange = { onMaxTrailLengthChange(it.roundToInt()) },
                            valueRange = 10f..1000f,
                            colors = SliderDefaults.colors(
                                thumbColor = Color(0xFF4CAF50),
                                activeTrackColor = Color(0xFF4CAF50)
                            )
                        )
                    }
                }
                ActiveControl.TRACKING_AREA -> {
                    val pct = (trackingPadding * 100).roundToInt()
                    Column(modifier = Modifier.padding(top = 4.dp)) {
                        Text(
                            "Extra Tracking Area: +${pct}% outside boundary",
                            color = Color.White.copy(alpha = 0.7f),
                            fontSize = 11.sp
                        )
                        Slider(
                            value = trackingPadding,
                            onValueChange = { onTrackingPaddingChange(it) },
                            valueRange = 0f..1f,
                            colors = SliderDefaults.colors(
                                thumbColor = Color(0xFF2196F3),
                                activeTrackColor = Color(0xFF2196F3)
                            )
                        )
                    }
                }
                else -> {}
            }
        }
    }
}

// ============================================================================
// Marker Data Panel
// ============================================================================

@Composable
fun MarkerDataPanel(markers: List<DetectedMarker>) {
    if (markers.isEmpty()) return
    val displayMarkers = markers.take(2)
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(Color(0xFF2A2A4A).copy(alpha = 0.95f))
            .padding(horizontal = 16.dp, vertical = 8.dp),
        horizontalArrangement = Arrangement.SpaceEvenly
    ) {
        displayMarkers.forEach { marker ->
            val color = TRAIL_COLORS[marker.id % TRAIL_COLORS.size]
            // Calculate coords with 0,0 at bottom right
            // centerX, centerY are 0..1 (top-left to bottom-right)
            val xMm = ((1f - marker.centerX) * FIELD_WIDTH_MM).roundToInt()
            val yMm = ((1f - marker.centerY) * FIELD_HEIGHT_MM).roundToInt()
            
            var angleDeg = 0
            if (marker.corners.size == 4) {
                val c0 = marker.corners[0]
                val c1 = marker.corners[1]
                val topMidX = (c0.x + c1.x) / 2f
                val topMidY = (c0.y + c1.y) / 2f
                val dx = topMidX - marker.centerX
                val dy = topMidY - marker.centerY
                var a = Math.toDegrees(kotlin.math.atan2(-dy.toDouble(), dx.toDouble())).toFloat()
                if (a < 0f) a += 360f
                angleDeg = a.roundToInt()
            }
            
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                Text(
                    text = "ID: ${marker.id}",
                    color = color,
                    fontWeight = FontWeight.Bold,
                    fontSize = 14.sp
                )
                Text(
                    text = "X: ${xMm} mm, Y: ${yMm} mm",
                    color = Color.White,
                    fontSize = 12.sp
                )
                Text(
                    text = "Direction: ${angleDeg}°",
                    color = Color.White.copy(alpha = 0.7f),
                    fontSize = 12.sp
                )
            }
        }
    }
}

// ============================================================================
// Main Screen
// ============================================================================

@Composable
fun PerspectiveWarpScreen() {
    val context = LocalContext.current
    val prefs = remember { context.getSharedPreferences("RobotPrefs", android.content.Context.MODE_PRIVATE) }
    
    var corners by remember {
        val def = listOf(
            Offset(0.15f, 0.15f),
            Offset(0.85f, 0.15f),
            Offset(0.85f, 0.85f),
            Offset(0.15f, 0.85f)
        )
        val loaded = mutableListOf<Offset>()
        var ok = true
        for (i in 0..3) {
            val x = prefs.getFloat("c${i}x", -100f)
            val y = prefs.getFloat("c${i}y", -100f)
            if (x == -100f || y == -100f) { ok = false; break }
            loaded.add(Offset(x, y))
        }
        mutableStateOf(if (ok) loaded else def)
    }

    var isLocked by remember { mutableStateOf(false) }
    var warpResult by remember { mutableStateOf<WarpResult?>(null) }
    var previewScale by remember { mutableFloatStateOf(0.70f) }
    var containerSize by remember { mutableStateOf(IntSize.Zero) }
    var cameraRect by remember { mutableStateOf(Rect.Zero) }

    // UDP bridge for robot communication
    val udpBridge: RobotUdpBridge = remember { RobotUdpBridge() }
    val connectedRobots by udpBridge.robots.collectAsState()
    val bridgeLogs by udpBridge.logs.collectAsState()
    val tapTargetState by udpBridge.tapTargetState.collectAsState()
    val headingLockState by udpBridge.headingLockState.collectAsState()
    var armedTapRobotId by remember { mutableStateOf<String?>(null) }

    // Start/stop the UDP listener with the composable lifecycle
    DisposableEffect(Unit) {
        udpBridge.start()
        onDispose { udpBridge.stop() }
    }

    // Trail tracking state — persists across lock/unlock
    var maxTrailLength by remember { mutableIntStateOf(200) }
    var trackingPadding by remember { mutableFloatStateOf(0f) }
    val trailMap = remember { mutableMapOf<Int, MutableList<Offset>>() }
    var trailVersion by remember { mutableIntStateOf(0) }

    val currentMaxTrailLength by rememberUpdatedState(maxTrailLength)

    Column(modifier = Modifier.fillMaxSize().background(Color(0xFF1A1A2E))) {
        // Top control bar
        ControlBar(
            maxTrailLength = maxTrailLength,
            onMaxTrailLengthChange = { maxTrailLength = it },
            trackingPadding = trackingPadding,
            onTrackingPaddingChange = { trackingPadding = it }
        )

        // Connection status + command bar
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .background(
                    if (connectedRobots.isEmpty()) Color(0xFF37474F)
                    else Color(0xFF1B5E20).copy(alpha = 0.9f)
                )
                .padding(horizontal = 12.dp, vertical = 6.dp)
        ) {
            if (connectedRobots.isEmpty()) {
                Text(
                    text = "Waiting for robot HELLO on UDP :4210 ...",
                    color = Color.White.copy(alpha = 0.6f),
                    fontSize = 12.sp
                )
            } else {
                for ((id, robot) in connectedRobots) {
                    val alive = udpBridge.isRobotConnected(id)
                    val markerVisible = warpResult?.markers?.any { MARKER_TO_ROBOT[it.id] == id } == true
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(8.dp),
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        // Status dot + label
                        Text(
                            text = if (alive) "\uD83D\uDFE2" else "\u26AA",
                            fontSize = 14.sp
                        )
                        Text(
                            text = "Robot $id @ ${robot.address.hostAddress}",
                            color = Color.White,
                            fontSize = 12.sp,
                            modifier = Modifier.weight(1f)
                        )
                        Button(
                            onClick = {
                                armedTapRobotId = null
                                udpBridge.startHeadingLock(id, 0f)
                            },
                            enabled = alive && tapTargetState?.active != true && headingLockState?.active != true,
                            modifier = Modifier.height(32.dp),
                            contentPadding = PaddingValues(horizontal = 10.dp, vertical = 0.dp),
                            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF6A1B9A))
                        ) {
                            Text(
                                if (headingLockState?.active == true && headingLockState?.robotId == id)
                                    "Locking 0°"
                                else "Lock 0°",
                                fontSize = 11.sp,
                                fontWeight = FontWeight.Bold
                            )
                        }
                        // Test Square button
                        Button(
                            onClick = {
                                armedTapRobotId = if (armedTapRobotId == id) null else id
                            },
                            enabled = isLocked && alive && markerVisible && headingLockState?.active != true,
                            modifier = Modifier.height(32.dp),
                            contentPadding = PaddingValues(horizontal = 10.dp, vertical = 0.dp),
                            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF1565C0))
                        ) {
                            Text(
                                when {
                                    armedTapRobotId == id -> "Tap Map"
                                    tapTargetState?.active == true && tapTargetState?.robotId == id -> "Retarget"
                                    else -> "Tap Target"
                                },
                                fontSize = 11.sp, fontWeight = FontWeight.Bold
                            )
                        }
                        // Abort button
                        Button(
                            onClick = {
                                if (armedTapRobotId == id) {
                                    armedTapRobotId = null
                                }
                                if (headingLockState?.active == true && headingLockState?.robotId == id) {
                                    udpBridge.cancelHeadingLock()
                                } else if (tapTargetState?.active == true && tapTargetState?.robotId == id) {
                                    udpBridge.cancelTapTarget()
                                } else {
                                    udpBridge.sendCommand(id, "A")
                                }
                            },
                            enabled = alive,
                            modifier = Modifier.height(32.dp),
                            contentPadding = PaddingValues(horizontal = 10.dp, vertical = 0.dp),
                            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFC62828))
                        ) {
                            Text("Abort", fontSize = 11.sp, fontWeight = FontWeight.Bold)
                        }
                    }
                }
                if (armedTapRobotId != null) {
                    Text(
                        text = "Tap the map to send Robot $armedTapRobotId while holding its current angle",
                        color = Color.White.copy(alpha = 0.75f),
                        fontSize = 11.sp,
                        modifier = Modifier.padding(top = 6.dp)
                    )
                } else if (tapTargetState?.active == true) {
                    Text(
                        text = "Robot ${tapTargetState?.robotId} target: (${tapTargetState?.targetXmm?.roundToInt()}, ${tapTargetState?.targetYmm?.roundToInt()}) @ ${tapTargetState?.holdAngleDeg?.roundToInt()}°",
                        color = Color.White.copy(alpha = 0.75f),
                        fontSize = 11.sp,
                        modifier = Modifier.padding(top = 6.dp)
                    )
                }
            }
        }

        // UDP debug log (last few lines, always visible while debugging)
        if (bridgeLogs.isNotEmpty()) {
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .background(Color(0xFF212121))
                    .padding(horizontal = 8.dp, vertical = 4.dp)
                    .heightIn(max = 80.dp)
            ) {
                bridgeLogs.take(5).forEach { line ->
                    Text(
                        text = line,
                        color = Color(0xFF80CBC4),
                        fontSize = 10.sp,
                        maxLines = 1
                    )
                }
            }
        }

        if (isLocked) {
            MarkerDataPanel(markers = warpResult?.markers ?: emptyList())
        }

        Box(
            modifier = Modifier
                .weight(1f)
                .fillMaxWidth()
                .onGloballyPositioned { coords ->
                    containerSize = coords.size
                },
            contentAlignment = Alignment.Center
        ) {
            // Camera preview inset
            Box(
                modifier = Modifier
                    .fillMaxWidth(previewScale)
                    .aspectRatio(9f / 16f)
                    .onGloballyPositioned { coords ->
                        val pos = coords.positionInParent()
                        cameraRect = Rect(
                            left = pos.x,
                            top = pos.y,
                            right = pos.x + coords.size.width,
                            bottom = pos.y + coords.size.height
                        )
                    }
            ) {
                CameraXPreview(
                    corners = corners,
                    isLocked = isLocked,
                    trackingPadding = trackingPadding,
                    onWarpResult = { result ->
                        warpResult = result
                        val detectedIds = result.markers.map { it.id }.toSet()
                        // Update trail map
                        result.markers.forEach { marker ->
                            val trail = trailMap.getOrPut(marker.id) { mutableListOf() }
                            trail.add(Offset(marker.centerX, marker.centerY))
                        }
                        
                        for ((id, trail) in trailMap) {
                            if (id !in detectedIds && trail.isNotEmpty()) {
                                trail.removeAt(0)
                            }
                            while (trail.size > currentMaxTrailLength) {
                                trail.removeAt(0)
                            }
                        }
                        trailMap.entries.removeAll { it.value.isEmpty() }
                        trailVersion++

                        // --- Send camera observations to connected robots ---
                        result.markers.forEach { marker ->
                            val robotId = MARKER_TO_ROBOT[marker.id] ?: return@forEach
                            if (!udpBridge.isRobotConnected(robotId)) return@forEach

                            // Convert normalized coords to mm (origin bottom-right)
                            val observation = markerObservation(marker)

                            // Because the phone is rotated on it's side while in portrait mode,
                            // The phone's XY coordinate system directly translates to YX on the robot's coordinate system
                            // We hereby swap X and Y to resolve this conflict
                            val xMm = observation.first
                            val yMm = observation.second

                            // Compute heading angle from ArUco corner geometry
                            val angleDeg = observation.third

                            udpBridge.sendCameraObservation(robotId, xMm, yMm, angleDeg, result.timestampMs)

                            udpBridge.updateHeadingLock(robotId, angleDeg)

                            // Update square test progress (phone-driven waypoint monitoring)
                            udpBridge.updateTapTarget(robotId, xMm, yMm, angleDeg)
                        }
                    }
                )

                if (isLocked) {
                    warpResult?.let { result ->
                        Box(
                            modifier = Modifier
                                .fillMaxWidth()
                                .aspectRatio(1f)
                                .align(Alignment.Center)
                        ) {
                            // Show warped bitmap
                            Image(
                                bitmap = result.bitmap.asImageBitmap(),
                                contentDescription = "Warped Frame",
                                modifier = Modifier.fillMaxSize()
                            )
                            // Overlay trails and current markers
                            TrailOverlay(
                                trailMap = trailMap,
                                currentMarkers = result.markers,
                                trailVersion = trailVersion
                            )
                            TapTargetOverlay(
                                activeTarget = tapTargetState,
                                armedRobotId = armedTapRobotId,
                                onTapNormalized = { target ->
                                    val robotId = armedTapRobotId
                                    val marker = if (robotId != null) {
                                        result.markers.find { MARKER_TO_ROBOT[it.id] == robotId }
                                    } else {
                                        null
                                    }
                                    if (robotId != null && marker != null) {
                                        val observation = markerObservation(marker)
                                        udpBridge.startTapTarget(
                                            robotId = robotId,
                                            targetXmm = (1f - target.x) * FIELD_WIDTH_MM,
                                            targetYmm = (1f - target.y) * FIELD_HEIGHT_MM,
                                            holdAngleDeg = observation.third
                                        )
                                    }
                                }
                            )
                        }
                    }
                }
            }

            // Draggable overlay (only when unlocked)
            if (!isLocked) {
                DraggableOverlay(
                    corners = corners,
                    cameraRect = cameraRect,
                    containerSize = containerSize,
                    onCornersChanged = { newCorners -> corners = newCorners },
                    onZoomChange = { factor -> previewScale = (previewScale * factor).coerceIn(0.3f, 1.0f) }
                )
            }
        }

        // Bottom controls
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .background(Color(0xFF16213E))
                .padding(16.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp, Alignment.CenterHorizontally),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Button(
                onClick = {
                    if (isLocked) {
                        armedTapRobotId = null
                        if (tapTargetState?.active == true) {
                            udpBridge.cancelTapTarget()
                        }
                        if (headingLockState?.active == true) {
                            udpBridge.cancelHeadingLock()
                        }
                    }
                    isLocked = !isLocked
                },
                modifier = Modifier.weight(1f).height(52.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = if (isLocked) Color(0xFF4CAF50) else Color(0xFFFF9800)
                )
            ) {
                Text(
                    if (isLocked) "🔓 Unlock Boundaries" else "🔒 Lock Map Boundaries",
                    fontWeight = FontWeight.Bold
                )
            }
            if (!isLocked) {
                Button(
                    onClick = {
                        val editor = prefs.edit()
                        corners.forEachIndexed { i, c ->
                            editor.putFloat("c${i}x", c.x)
                            editor.putFloat("c${i}y", c.y)
                        }
                        editor.apply()
                    },
                    modifier = Modifier.height(52.dp),
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF2196F3))
                ) {
                    Text("💾 Save", fontWeight = FontWeight.Bold)
                }
            }
        }
    }
}

// ============================================================================
// Trail Overlay (shown only when locked)
// ============================================================================

@Composable
fun TrailOverlay(
    trailMap: Map<Int, MutableList<Offset>>,
    currentMarkers: List<DetectedMarker>,
    trailVersion: Int
) {
    val textMeasurer = rememberTextMeasurer()
    // Read trailVersion to register as a compose dependency
    val currentVersion = trailVersion

    Canvas(modifier = Modifier.fillMaxSize()) {
        val w = size.width
        val h = size.height
        val currentMarkerIds = currentMarkers.map { it.id }.toSet()

        trailMap.forEach { (markerId, trail) ->
            if (trail.isEmpty()) return@forEach

            val color = TRAIL_COLORS[markerId % TRAIL_COLORS.size]

            // Draw trail polyline with fading opacity
            for (i in 1 until trail.size) {
                val alpha = (i.toFloat() / trail.size).coerceIn(0.05f, 0.9f)
                val from = Offset(trail[i - 1].x * w, trail[i - 1].y * h)
                val to = Offset(trail[i].x * w, trail[i].y * h)
                drawLine(
                    color = color.copy(alpha = alpha),
                    start = from,
                    end = to,
                    strokeWidth = 3f
                )
            }

            if (markerId in currentMarkerIds) {
                // Draw current position — outer ring + inner dot
                val last = trail.last()
                val cx = last.x * w
                val cy = last.y * h
                drawCircle(color, 12f, Offset(cx, cy))
                drawCircle(Color.White, 6f, Offset(cx, cy))

                // Draw orientation vector
                val marker = currentMarkers.find { it.id == markerId }
                if (marker != null && marker.corners.size == 4) {
                    val c0 = marker.corners[0]
                    val c1 = marker.corners[1]
                    val topMidX = (c0.x + c1.x) / 2f * w
                    val topMidY = (c0.y + c1.y) / 2f * h
                    
                    val dx = topMidX - cx
                    val dy = topMidY - cy
                    val length = sqrt(dx * dx + dy * dy)
                    
                    if (length > 0) {
                        // Extend pointer 1.5x the distance to the edge, but at least 40px
                        val pointerLen = (length * 1.5f).coerceAtLeast(40f)
                        val endX = cx + (dx / length) * pointerLen
                        val endY = cy + (dy / length) * pointerLen
                        
                        drawLine(
                            color = Color.White,
                            start = Offset(cx, cy),
                            end = Offset(endX, endY),
                            strokeWidth = 8f,
                            cap = androidx.compose.ui.graphics.StrokeCap.Round
                        )
                        drawLine(
                            color = color,
                            start = Offset(cx, cy),
                            end = Offset(endX, endY),
                            strokeWidth = 4f,
                            cap = androidx.compose.ui.graphics.StrokeCap.Round
                        )
                    }
                }

                // ID label
                val label = "ID:$markerId"
                val textResult = textMeasurer.measure(
                    text = label,
                    style = TextStyle(
                        color = Color.White,
                        fontSize = 13.sp,
                        fontWeight = FontWeight.Bold,
                        background = color.copy(alpha = 0.7f)
                    )
                )
                drawText(
                    textLayoutResult = textResult,
                    topLeft = Offset(cx + 16f, cy - textResult.size.height / 2f)
                )
            }
        }
    }
}

@Composable
fun TapTargetOverlay(
    activeTarget: TapTargetState?,
    armedRobotId: String?,
    onTapNormalized: (Offset) -> Unit
) {
    Canvas(
        modifier = Modifier
            .fillMaxSize()
            .pointerInput(armedRobotId) {
                detectTapGestures { position ->
                    if (armedRobotId == null) return@detectTapGestures
                    val width = size.width.toFloat()
                    val height = size.height.toFloat()
                    if (width <= 0f || height <= 0f) return@detectTapGestures
                    onTapNormalized(
                        Offset(
                            x = (position.x / width).coerceIn(0f, 1f),
                            y = (position.y / height).coerceIn(0f, 1f)
                        )
                    )
                }
            }
    ) {
        val target = activeTarget
        if (target == null || !target.active) return@Canvas

        val cx = (1f - (target.targetXmm / FIELD_WIDTH_MM)).coerceIn(0f, 1f) * size.width
        val cy = (1f - (target.targetYmm / FIELD_HEIGHT_MM)).coerceIn(0f, 1f) * size.height

        drawCircle(
            color = Color(0x66FF5252),
            radius = 24f,
            center = Offset(cx, cy)
        )
        drawCircle(
            color = Color(0xFFFF5252),
            radius = 10f,
            center = Offset(cx, cy),
            style = Stroke(width = 3f)
        )
        drawLine(
            color = Color(0xFFFF5252),
            start = Offset(cx - 24f, cy),
            end = Offset(cx + 24f, cy),
            strokeWidth = 3f
        )
        drawLine(
            color = Color(0xFFFF5252),
            start = Offset(cx, cy - 24f),
            end = Offset(cx, cy + 24f),
            strokeWidth = 3f
        )
    }
}

// ============================================================================
// CameraX Preview + Analysis
// ============================================================================

@OptIn(ExperimentalCamera2Interop::class)
@Composable
fun CameraXPreview(
    corners: List<Offset>,
    isLocked: Boolean,
    trackingPadding: Float,
    onWarpResult: (WarpResult) -> Unit
) {
    val lifecycleOwner = LocalLifecycleOwner.current
    val context = LocalContext.current
    val cameraProviderFuture = remember { ProcessCameraProvider.getInstance(context) }

    val currentIsLocked by rememberUpdatedState(isLocked)
    val currentCorners by rememberUpdatedState(corners)
    val currentTrackingPadding by rememberUpdatedState(trackingPadding)
    val currentOnWarpResult by rememberUpdatedState(onWarpResult)

    AndroidView(
        factory = { ctx ->
            val previewView = PreviewView(ctx).apply {
                scaleType = PreviewView.ScaleType.FIT_CENTER
            }

            cameraProviderFuture.addListener({
                val cameraProvider = cameraProviderFuture.get()

                val preview = Preview.Builder()
                    .setTargetAspectRatio(AspectRatio.RATIO_16_9)
                    .build().also {
                        it.setSurfaceProvider(previewView.surfaceProvider)
                    }

                val backgroundExecutor = java.util.concurrent.Executors.newSingleThreadExecutor()
                val imageAnalysis = ImageAnalysis.Builder()
                    .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                    .setTargetAspectRatio(AspectRatio.RATIO_16_9)
                    .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                    .build()
                    .also {
                        it.setAnalyzer(backgroundExecutor) { imageProxy ->
                            if (currentIsLocked) {
                                val startTime = System.currentTimeMillis()
                                val result = warpAndDetect(
                                    imageProxy,
                                    currentCorners,
                                    currentTrackingPadding,
                                    startTime
                                )
                                if (result != null) {
                                    ContextCompat.getMainExecutor(ctx).execute {
                                        currentOnWarpResult(result)
                                    }
                                }
                            }
                            imageProxy.close()
                        }
                    }

                val cameraSelector = try {
                    val backCameraIds = cameraProvider.availableCameraInfos.filter {
                        it.lensFacing == CameraSelector.LENS_FACING_BACK
                    }
                    
                    // The Ultra Wide lens typically has the largest Field of View (FOV)
                    // or we can look for specific physical characteristics if needed.
                    // On many devices, it's identified by a smaller focal length.
                    val ultraWide = backCameraIds.firstOrNull { info ->
                        val characteristics = Camera2CameraInfo.from(info).getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
                        val focalLengths = Camera2CameraInfo.from(info).getCameraCharacteristic(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
                        
                        // Ultra wide usually has focal length < 2.5mm or so on phones
                        focalLengths?.any { it < 2.5f } == true
                    } ?: backCameraIds.lastOrNull() // Fallback to last back camera which is often the UW on multi-cam setups

                    if (ultraWide != null) {
                        CameraSelector.Builder()
                            .addCameraFilter { cameraInfos ->
                                cameraInfos.filter { it == ultraWide }
                            }
                            .build()
                    } else {
                        CameraSelector.DEFAULT_BACK_CAMERA
                    }
                } catch (e: Exception) {
                    CameraSelector.DEFAULT_BACK_CAMERA
                }

                try {
                    cameraProvider.unbindAll()
                    val camera = cameraProvider.bindToLifecycle(
                        lifecycleOwner, cameraSelector, preview, imageAnalysis
                    )
                    camera.cameraControl.setLinearZoom(0f)
                } catch (e: Exception) {
                    e.printStackTrace()
                }
            }, ContextCompat.getMainExecutor(ctx))

            previewView
        },
        modifier = Modifier.fillMaxSize()
    )
}

// ============================================================================
// Image Warping + ArUco Detection (with padding support)
// ============================================================================

private val arucoDict = Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50)
private val arucoParams = DetectorParameters()
private val arucoDetector = ArucoDetector(arucoDict, arucoParams)

@androidx.annotation.OptIn(androidx.camera.core.ExperimentalGetImage::class)
fun warpAndDetect(
    imageProxy: ImageProxy,
    corners: List<Offset>,
    paddingFraction: Float,
    captureTimeMs: Long
): WarpResult? {
    try {
        val plane = imageProxy.planes[0]
        val buffer = plane.buffer
        val rowStride = plane.rowStride

        val bitmap = Bitmap.createBitmap(rowStride / 4, imageProxy.height, Bitmap.Config.ARGB_8888)
        buffer.rewind()
        bitmap.copyPixelsFromBuffer(buffer)

        val croppedBitmap = if (rowStride / 4 > imageProxy.width) {
            Bitmap.createBitmap(bitmap, 0, 0, imageProxy.width, imageProxy.height)
        } else {
            bitmap
        }

        val rotatedBitmap = if (imageProxy.imageInfo.rotationDegrees != 0) {
            val matrix = android.graphics.Matrix()
            matrix.postRotate(imageProxy.imageInfo.rotationDegrees.toFloat())
            Bitmap.createBitmap(croppedBitmap, 0, 0, croppedBitmap.width, croppedBitmap.height, matrix, true)
        } else {
            croppedBitmap
        }

        val mat = Mat()
        Utils.bitmapToMat(rotatedBitmap, mat)

        // --- Extended warp with padding ---
        val baseSize = 1000.0
        val maxPaddingPx = 250.0 // max 250px padding per side → 1500 total max
        val padding = paddingFraction * maxPaddingPx
        val totalSize = baseSize + 2.0 * padding

        val w = rotatedBitmap.width.toDouble()
        val h = rotatedBitmap.height.toDouble()

        // Source corners in original image
        val srcMat = Mat(4, 1, CvType.CV_32FC2)
        srcMat.put(
            0, 0,
            corners[0].x * w, corners[0].y * h,
            corners[1].x * w, corners[1].y * h,
            corners[2].x * w, corners[2].y * h,
            corners[3].x * w, corners[3].y * h
        )

        // Destination: boundary region maps to center of padded output
        val dstMat = Mat(4, 1, CvType.CV_32FC2)
        dstMat.put(
            0, 0,
            padding, padding,
            padding + baseSize, padding,
            padding + baseSize, padding + baseSize,
            padding, padding + baseSize
        )

        val perspectiveTransform = Imgproc.getPerspectiveTransform(srcMat, dstMat)
        val warpedMat = Mat()
        Imgproc.warpPerspective(
            mat, warpedMat, perspectiveTransform,
            org.opencv.core.Size(totalSize, totalSize),
            Imgproc.INTER_LINEAR,
            org.opencv.core.Core.BORDER_CONSTANT
        )

        // --- ArUco Detection on the full padded image ---
        val grayMat = Mat()
        Imgproc.cvtColor(warpedMat, grayMat, Imgproc.COLOR_RGBA2GRAY)

        val detectedCorners = ArrayList<Mat>()
        val ids = Mat()
        arucoDetector.detectMarkers(grayMat, detectedCorners, ids)

        val markers = mutableListOf<DetectedMarker>()
        if (ids.rows() > 0) {
            val idsArray = IntArray(ids.total().toInt())
            ids.get(0, 0, idsArray)

            for (i in idsArray.indices) {
                val markerId = idsArray[i]
                val cornerMat = detectedCorners[i]

                val markerCorners = mutableListOf<Offset>()
                var cx = 0.0
                var cy = 0.0
                for (j in 0..3) {
                    val px = cornerMat.get(0, j)
                    // Normalize relative to boundary region
                    val cornerX = ((px[0] - padding) / baseSize).toFloat()
                    val cornerY = ((px[1] - padding) / baseSize).toFloat()
                    markerCorners.add(Offset(cornerX, cornerY))
                    cx += px[0]
                    cy += px[1]
                }
                cx /= 4.0
                cy /= 4.0

                markers.add(
                    DetectedMarker(
                        id = markerId,
                        centerX = ((cx - padding) / baseSize).toFloat(),
                        centerY = ((cy - padding) / baseSize).toFloat(),
                        corners = markerCorners
                    )
                )
            }
        }

        // --- Crop the boundary region for display ---
        val pInt = padding.toInt()
        val bInt = baseSize.toInt()
        val boundaryRect = org.opencv.core.Rect(pInt, pInt, bInt, bInt)
        val croppedWarpMat = Mat(warpedMat, boundaryRect)

        val finalRGB = Mat()
        Imgproc.cvtColor(croppedWarpMat, finalRGB, Imgproc.COLOR_RGBA2RGB)

        val resultBmp = Bitmap.createBitmap(bInt, bInt, Bitmap.Config.ARGB_8888)
        Utils.matToBitmap(finalRGB, resultBmp)

        // Cleanup
        mat.release()
        srcMat.release()
        dstMat.release()
        perspectiveTransform.release()
        warpedMat.release()
        grayMat.release()
        croppedWarpMat.release()
        finalRGB.release()
        ids.release()
        detectedCorners.forEach { it.release() }

        return WarpResult(bitmap = resultBmp, markers = markers, timestampMs = captureTimeMs)
    } catch (e: Exception) {
        e.printStackTrace()
        return null
    }
}

// ============================================================================
// Draggable Corner Overlay
// ============================================================================

@Composable
fun DraggableOverlay(
    corners: List<Offset>,
    cameraRect: Rect,
    containerSize: IntSize,
    onCornersChanged: (List<Offset>) -> Unit,
    onZoomChange: (Float) -> Unit
) {
    val currentCorners by rememberUpdatedState(corners)
    val currentCameraRect by rememberUpdatedState(cameraRect)

    fun cornerToScreen(c: Offset): Offset {
        val camW = currentCameraRect.width
        val camH = currentCameraRect.height
        return Offset(
            x = currentCameraRect.left + c.x * camW,
            y = currentCameraRect.top + c.y * camH
        )
    }

    fun screenDeltaToCameraDelta(dx: Float, dy: Float): Offset {
        val camW = currentCameraRect.width
        val camH = currentCameraRect.height
        if (camW == 0f || camH == 0f) return Offset.Zero
        return Offset(dx / camW, dy / camH)
    }

    Canvas(
        modifier = Modifier
            .fillMaxSize()
            .pointerInput(Unit) {
                awaitEachGesture {
                    val firstDown = awaitFirstDown(requireUnconsumed = false)

                    var draggedIdx = -1
                    var nearestIdx = 0
                    var minDist = Float.MAX_VALUE
                    currentCorners.forEachIndexed { i, corner ->
                        val screenPos = cornerToScreen(corner)
                        val d = sqrt(
                            (firstDown.position.x - screenPos.x) * (firstDown.position.x - screenPos.x) +
                                    (firstDown.position.y - screenPos.y) * (firstDown.position.y - screenPos.y)
                        )
                        if (d < minDist) {
                            minDist = d
                            nearestIdx = i
                        }
                    }
                    draggedIdx = if (minDist < 150f) nearestIdx else -1

                    var previousPosition = firstDown.position
                    var previousSpread = 0f
                    var prevPointerCount = 1

                    do {
                        val event = awaitPointerEvent()
                        val pressed = event.changes.filter { it.pressed }

                        if (pressed.size == 1 && draggedIdx != -1) {
                            val change = pressed[0]
                            val delta = change.position - previousPosition
                            if (delta != Offset.Zero) {
                                val cameraDelta = screenDeltaToCameraDelta(delta.x, delta.y)
                                val mutList = currentCorners.toMutableList()
                                mutList[draggedIdx] = Offset(
                                    x = mutList[draggedIdx].x + cameraDelta.x,
                                    y = mutList[draggedIdx].y + cameraDelta.y
                                )
                                onCornersChanged(mutList)
                            }
                            previousPosition = change.position
                            change.consume()
                        } else if (pressed.size >= 2) {
                            val p1 = pressed[0].position
                            val p2 = pressed[1].position
                            val spread = sqrt(
                                (p1.x - p2.x) * (p1.x - p2.x) +
                                        (p1.y - p2.y) * (p1.y - p2.y)
                            )
                            if (prevPointerCount >= 2 && previousSpread > 0f) {
                                val zoomFactor = spread / previousSpread
                                onZoomChange(zoomFactor)
                            }
                            previousSpread = spread
                            pressed.forEach { it.consume() }
                        }
                        prevPointerCount = pressed.size
                    } while (event.changes.any { it.pressed })
                }
            }
    ) {
        val camRect = currentCameraRect

        drawRect(
            color = Color.White.copy(alpha = 0.3f),
            topLeft = Offset(camRect.left, camRect.top),
            size = androidx.compose.ui.geometry.Size(camRect.width, camRect.height),
            style = Stroke(width = 2f)
        )

        val pts = corners.map { cornerToScreen(it) }

        val strokeColor = Color.Yellow
        val pinColor = Color.Red.copy(alpha = 0.4f)

        drawLine(strokeColor, pts[0], pts[1], 4f)
        drawLine(strokeColor, pts[1], pts[2], 4f)
        drawLine(strokeColor, pts[2], pts[3], 4f)
        drawLine(strokeColor, pts[3], pts[0], 4f)

        pts.forEach { pt ->
            drawCircle(pinColor, 40f, pt)
            // Crosshair
            drawLine(Color.White.copy(alpha = 0.8f), Offset(pt.x - 15f, pt.y), Offset(pt.x + 15f, pt.y), 2f)
            drawLine(Color.White.copy(alpha = 0.8f), Offset(pt.x, pt.y - 15f), Offset(pt.x, pt.y + 15f), 2f)
            // Center dot
            drawCircle(Color.White, 3f, pt)
        }
    }
}