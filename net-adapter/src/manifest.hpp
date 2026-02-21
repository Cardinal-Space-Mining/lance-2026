#pragma once

// Helper to expand a single Talon motor into its sub-topics
#define TALON_FEEDBACK(ENTRY, base_topic, slug)                             \
    ENTRY(base_topic "/ctrl", TalonFaultsAdapter, slug##_ctrl, INCOMING)    \
    ENTRY(base_topic "/info", TalonInfoAdapter, slug##_info, OUTGOING)      \
    ENTRY(base_topic "/faults", TalonFaultsAdapter, slug##_fault, OUTGOING)

// THE MASTER LIST
// Format: ENTRY(Topic, AdapterClass, DirectionFromRobotPerspective)
#define NETWORK_MANIFEST(ENTRY)                                                \
    /* Control (Client -> Robot) */                                            \
    ENTRY("/joy", JoyAdapter, joy, INCOMING)                                   \
    ENTRY("lance/watchdog_status", StdInt32Adapter, watchdog_status, INCOMING) \
    ENTRY("/clicked_point", PointStampedAdapter, clicked_point, INCOMING)      \
                                                                               \
    /* Data (Robot -> Client) */                                               \
    ENTRY("multiscan/imu", MS136ImuAdapter, imu, OUTGOING)                     \
    ENTRY("multiscan/lidar_scan", MS136ScanAdapter, lidar_scan, OUTGOING)      \
    ENTRY("cardinal_perception/planned_path", PathAdapter, path, OUTGOING)     \
    ENTRY("lance/relay_status", StdInt8Adapter, relay_status, OUTGOING)        \
    ENTRY("lance/op_status", StdStringAdapter, op_status, OUTGOING)            \
                                                                               \
    /* Talon Motors */                                                         \
    TALON_FEEDBACK(ENTRY, "lance/track_left", track_left)                      \
    TALON_FEEDBACK(ENTRY, "lance/track_right", track_right)                    \
    TALON_FEEDBACK(ENTRY, "lance/trencher", trencher)                          \
    TALON_FEEDBACK(ENTRY, "lance/hopper_belt", hopper_belt)                    \
    TALON_FEEDBACK(ENTRY, "lance/hopper_act", hopper_act)
