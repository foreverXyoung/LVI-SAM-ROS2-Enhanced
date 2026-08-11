#!/usr/bin/env python3
"""Validate LVI-SAM configuration files without starting ROS nodes."""

import argparse
import math
import re
import sys
from pathlib import Path

try:
    import yaml
except ImportError as exc:
    print(
        "PyYAML is required; install the rosdep key python3-yaml or apt package "
        "python3-yaml",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


class UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects silently shadowed parameter keys."""


def construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"found duplicate key {key!r}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    construct_unique_mapping,
)


LIDAR_CONFIGS = (
    "params_mapping.yaml",
    "params_localization.yaml",
    "params_charging_mapping.yaml",
    "params_charging_localization.yaml",
    "params_gazebo_mapping.yaml",
    "params_gazebo_localization.yaml",
)
LEGACY_LIDAR_CONFIG = "params.yaml"
IMU_PROFILES = (
    "params_imu_external.yaml",
    "params_imu_mid360.yaml",
)
MOUNT_PROFILES = ("params_mount_robot.yaml",)
SHARED_INTERFACE_KEYS = (
    "pointCloudTopic",
    "imuTopic",
    "odomTopic",
    "lidarFrame",
    "baselinkFrame",
    "odometryFrame",
    "mapFrame",
    "sensor",
)
SCENE_MODE_PAIRS = (
    ("params_mapping.yaml", "params_localization.yaml"),
    ("params_charging_mapping.yaml", "params_charging_localization.yaml"),
    ("params_gazebo_mapping.yaml", "params_gazebo_localization.yaml"),
)
SCENE_SENSOR_KEYS = (
    "pointCloudTopic",
    "imuTopic",
    "odomTopic",
    "lidarFrame",
    "baselinkFrame",
    "odometryFrame",
    "mapFrame",
    "sensor",
    "N_SCAN",
    "Horizon_SCAN",
    "downsampleRate",
    "extrinsicRot",
    "extrinsicRPY",
    "extrinsicTrans",
    "imuAccelerationScale",
    "imuAccNoise",
    "imuGyrNoise",
    "imuAccBiasN",
    "imuGyrBiasN",
    "imuGravity",
    "imuRPYWeight",
)

IMU_PROFILE_VIS_KEYS = (
    "imu_topic",
    "imuAccelerationScale",
    "acc_n",
    "gyr_n",
    "acc_w",
    "gyr_w",
    "g_norm",
)


def load_parameters(path: Path) -> dict:
    try:
        document = yaml.load(
            path.read_text(encoding="utf-8"), Loader=UniqueKeyLoader
        )
        return document["/**"]["ros__parameters"]
    except (OSError, UnicodeError, yaml.YAMLError, KeyError, TypeError) as exc:
        raise ValueError(f"cannot read ROS parameter YAML {path}: {exc}") from exc


def finite_number(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def normalize_topic(topic: str) -> str:
    if not isinstance(topic, str):
        return ""
    return "/" + topic.strip().strip("/")


def validate_vector(errors, source, params, key, size):
    value = params.get(key)
    if not isinstance(value, list) or len(value) != size or not all(
        finite_number(item) for item in value
    ):
        errors.append(f"{source}: {key} must contain {size} finite numbers")


def validate_rotation(errors, source, params, key):
    values = params.get(key)
    if not isinstance(values, list) or len(values) != 9 or not all(
        finite_number(item) for item in values
    ):
        return
    rows = [values[0:3], values[3:6], values[6:9]]
    tolerance = 1.0e-3
    for row_index in range(3):
        for column_index in range(3):
            dot = sum(
                float(rows[k][row_index]) * float(rows[k][column_index])
                for k in range(3)
            )
            expected = 1.0 if row_index == column_index else 0.0
            if abs(dot - expected) > tolerance:
                errors.append(f"{source}: {key} must be an orthonormal rotation")
                return
    determinant = (
        float(values[0])
        * (float(values[4]) * float(values[8]) - float(values[5]) * float(values[7]))
        - float(values[1])
        * (float(values[3]) * float(values[8]) - float(values[5]) * float(values[6]))
        + float(values[2])
        * (float(values[3]) * float(values[7]) - float(values[4]) * float(values[6]))
    )
    if abs(determinant - 1.0) > tolerance:
        errors.append(f"{source}: {key} must have determinant +1")


def resolve_asset(config_dir: Path, configured_path: str) -> Path:
    requested = Path(configured_path)
    if requested.is_absolute() and requested.is_file():
        return requested
    return config_dir.parent / configured_path.lstrip("/")


def validate_lidar_configs(config_dir: Path, errors: list[str]) -> dict:
    loaded = {}
    for name in LIDAR_CONFIGS:
        path = config_dir / name
        try:
            params = load_parameters(path)
        except ValueError as exc:
            errors.append(str(exc))
            continue
        loaded[name] = params

        loc = params.get("Loc")
        expected_localization = name.endswith("localization.yaml")
        if not isinstance(loc, dict) or loc.get("EnableFlag") is not expected_localization:
            errors.append(
                f"{name}: Loc.EnableFlag must be {str(expected_localization).lower()}"
            )
        if isinstance(loc, dict) and not expected_localization:
            unused_mapping_loc_keys = set(loc) - {"EnableFlag"}
            if unused_mapping_loc_keys:
                errors.append(
                    f"{name}: mapping Loc group must contain only EnableFlag; "
                    "move localization-only keys to the paired localization profile: "
                    + ", ".join(sorted(unused_mapping_loc_keys))
                )
        if not isinstance(params.get("useRviz"), bool):
            errors.append(f"{name}: useRviz must be a top-level boolean")
        if isinstance(loc, dict) and "useRviz" in loc:
            errors.append(f"{name}: useRviz must not be nested under Loc")
        expected_map_saving = not expected_localization
        if params.get("savePCD") is not expected_map_saving:
            errors.append(
                f"{name}: savePCD must be "
                f"{str(expected_map_saving).lower()} for this mode"
            )
        for key in SHARED_INTERFACE_KEYS:
            if not isinstance(params.get(key), str) or not params[key].strip():
                errors.append(f"{name}: missing non-empty interface parameter {key}")
        if params.get("sensor") not in ("livox", "velodyne", "ouster"):
            errors.append(f"{name}: sensor must be livox, velodyne or ouster")
        for key in ("pointCloudTopic", "imuTopic", "odomTopic"):
            if isinstance(params.get(key), str) and not params[key].startswith("/"):
                errors.append(f"{name}: {key} must be an absolute topic")
        validate_vector(errors, name, params, "extrinsicRot", 9)
        validate_vector(errors, name, params, "extrinsicRPY", 9)
        validate_vector(errors, name, params, "extrinsicTrans", 3)
        validate_rotation(errors, name, params, "extrinsicRot")
        validate_rotation(errors, name, params, "extrinsicRPY")
        for key in ("gpsCovThreshold", "gpsVarianceFloor", "gpsTimeTolerance"):
            if not finite_number(params.get(key)) or float(params[key]) <= 0.0:
                errors.append(f"{name}: {key} must be finite and greater than 0")
        if not isinstance(params.get("gpsQueueSize"), int) or params["gpsQueueSize"] <= 0:
            errors.append(f"{name}: gpsQueueSize must be a positive integer")
        for key in ("edgeFeatureMinValidNum", "surfFeatureMinValidNum"):
            if not isinstance(params.get(key), int) or params[key] < 0:
                errors.append(f"{name}: {key} must be a non-negative integer")
        for key in ("edgeThreshold", "surfThreshold"):
            if not finite_number(params.get(key)) or float(params[key]) <= 0.0:
                errors.append(f"{name}: {key} must be finite and greater than 0")
        for key in ("N_SCAN", "Horizon_SCAN", "downsampleRate", "numberOfCores"):
            if not isinstance(params.get(key), int) or params[key] <= 0:
                errors.append(f"{name}: {key} must be a positive integer")
        for key in (
            "imuAccNoise", "imuGyrNoise", "imuAccBiasN", "imuGyrBiasN",
            "imuGravity", "imuAccelerationScale", "odometrySurfLeafSize",
            "mappingCornerLeafSize",
            "mappingSurfLeafSize", "surroundingkeyframeAddingDistThreshold",
            "surroundingkeyframeAddingAngleThreshold",
            "surroundingKeyframeDensity", "surroundingKeyframeSearchRadius",
            "globalMapVisualizationSearchRadius",
            "globalMapVisualizationPoseDensity",
            "globalMapVisualizationLeafSize",
        ):
            if not finite_number(params.get(key)) or float(params[key]) <= 0.0:
                errors.append(f"{name}: {key} must be finite and greater than 0")
        for key in ("imuRPYWeight", "mappingProcessInterval", "z_tollerance",
                    "rotation_tollerance"):
            if not finite_number(params.get(key)) or float(params[key]) < 0.0:
                errors.append(f"{name}: {key} must be finite and non-negative")
        minimum_range = params.get("lidarMinRange")
        maximum_range = params.get("lidarMaxRange")
        if (
            not finite_number(minimum_range)
            or not finite_number(maximum_range)
            or float(minimum_range) < 0.0
            or float(maximum_range) <= float(minimum_range)
        ):
            errors.append(
                f"{name}: lidarMaxRange must exceed non-negative lidarMinRange"
            )
        scan_context_threshold = params.get("scanContextDistanceThreshold")
        if (
            not finite_number(scan_context_threshold)
            or not 0.0 < float(scan_context_threshold) < 1.0
        ):
            errors.append(
                f"{name}: scanContextDistanceThreshold must be in (0, 1)"
            )
        if params.get("loopClosureEnableFlag") is True:
            for key in (
                "loopClosureFrequency", "historyKeyframeSearchRadius",
                "historyKeyframeSearchTimeDiff", "historyKeyframeFitnessScore",
                "externalLoopTimeTolerance",
            ):
                if not finite_number(params.get(key)) or float(params[key]) <= 0.0:
                    errors.append(
                        f"{name}: enabled loop closure requires positive {key}"
                    )
            for key in ("surroundingKeyframeSize", "historyKeyframeSearchNum"):
                if not isinstance(params.get(key), int) or params[key] <= 0:
                    errors.append(
                        f"{name}: enabled loop closure requires positive {key}"
                    )
        if not finite_number(params.get("externalLoopTimeTolerance")) or float(
            params["externalLoopTimeTolerance"]
        ) <= 0.0:
            errors.append(
                f"{name}: externalLoopTimeTolerance must be finite and greater than 0"
            )
        if params.get("useGpsFactor") is True and not str(
            params.get("gpsExpectedFrame", "")
        ).strip():
            errors.append(f"{name}: gpsExpectedFrame is required when useGpsFactor=true")
        if isinstance(loc, dict):
            use_rtk = loc.get("useRTKAssist", False)
            use_rtk_initialization = loc.get("useRTKInitialization", False)
            if use_rtk_initialization and not use_rtk:
                errors.append(
                    f"{name}: Loc.useRTKInitialization requires Loc.useRTKAssist=true"
                )
            if use_rtk and not str(loc.get("rtkExpectedFrame", "")).strip():
                errors.append(
                    f"{name}: Loc.rtkExpectedFrame is required when RTK assist is enabled"
                )
            for key in ("rtkMaxInnovation", "rtkMaxNormalizedInnovation"):
                if key in loc and (
                    not finite_number(loc[key]) or float(loc[key]) <= 0.0
                ):
                    errors.append(f"{name}: Loc.{key} must be finite and greater than 0")

    if loaded:
        reference_name = LIDAR_CONFIGS[0]
        reference = loaded.get(reference_name, next(iter(loaded.values())))
        for name, params in loaded.items():
            for key in SHARED_INTERFACE_KEYS:
                if key in reference and key in params and params[key] != reference[key]:
                    errors.append(
                        f"{name}: {key}={params[key]!r} differs from shared contract "
                        f"{reference[key]!r}"
                    )

        for mapping_name, localization_name in SCENE_MODE_PAIRS:
            mapping = loaded.get(mapping_name)
            localization = loaded.get(localization_name)
            if mapping is None or localization is None:
                continue
            for key in SCENE_SENSOR_KEYS:
                if mapping.get(key) != localization.get(key):
                    errors.append(
                        f"{localization_name}: {key} differs from its mapping "
                        f"profile {mapping_name}; one scene must use one sensor "
                        "and calibration contract"
                    )

        try:
            legacy = load_parameters(config_dir / LEGACY_LIDAR_CONFIG)
        except ValueError as exc:
            errors.append(str(exc))
        else:
            reference = loaded.get("params_localization.yaml", reference)
            if not isinstance(legacy.get("Loc"), dict) or legacy["Loc"].get(
                "EnableFlag"
            ) is not True:
                errors.append(
                    f"{LEGACY_LIDAR_CONFIG}: Loc.EnableFlag must remain true"
                )
            if not isinstance(legacy.get("useRviz"), bool):
                errors.append(
                    f"{LEGACY_LIDAR_CONFIG}: useRviz must be a top-level boolean"
                )
            if isinstance(legacy.get("Loc"), dict) and "useRviz" in legacy["Loc"]:
                errors.append(
                    f"{LEGACY_LIDAR_CONFIG}: useRviz must not be nested under Loc"
                )
            for key in SCENE_SENSOR_KEYS:
                if legacy.get(key) != reference.get(key):
                    errors.append(
                        f"{LEGACY_LIDAR_CONFIG}: legacy {key} must match "
                        "params_localization.yaml"
                    )
    return loaded


def validate_imu_profiles(config_dir: Path, errors: list[str]) -> dict:
    loaded = {}
    for name in IMU_PROFILES:
        try:
            params = load_parameters(config_dir / name)
        except ValueError as exc:
            errors.append(str(exc))
            continue
        loaded[name] = params

        for key in ("imuTopic", "imu_topic"):
            value = params.get(key)
            if not isinstance(value, str) or not value.startswith("/"):
                errors.append(f"{name}: {key} must be an absolute topic")
        if params.get("imuTopic") != params.get("imu_topic"):
            errors.append(f"{name}: imuTopic and imu_topic must be identical")

        for key in (
            "imuAccelerationScale", "imuAccNoise", "imuGyrNoise",
            "imuAccBiasN", "imuGyrBiasN", "imuGravity", "acc_n",
            "gyr_n", "acc_w", "gyr_w", "g_norm",
        ):
            if not finite_number(params.get(key)) or float(params[key]) <= 0.0:
                errors.append(f"{name}: {key} must be finite and greater than 0")
        if not finite_number(params.get("imuRPYWeight")) or float(
            params["imuRPYWeight"]
        ) < 0.0:
            errors.append(f"{name}: imuRPYWeight must be finite and non-negative")

        validate_vector(errors, name, params, "imuToLidarRotation", 9)
        validate_vector(
            errors, name, params, "imuOrientationToLidarRotation", 9
        )
        validate_vector(errors, name, params, "imuToLidarTranslation", 3)
        validate_rotation(errors, name, params, "imuToLidarRotation")
        validate_rotation(
            errors, name, params, "imuOrientationToLidarRotation"
        )
        if params.get("imuOrientationSource") not in ("message", "mount"):
            errors.append(
                f"{name}: imuOrientationSource must be message or mount"
            )
        if (
            params.get("imuOrientationSource") == "mount"
            and params.get("imuRPYWeight") != 0.0
        ):
            errors.append(
                f"{name}: mount orientation source requires imuRPYWeight=0.0"
            )

    external = loaded.get("params_imu_external.yaml")
    mid360 = loaded.get("params_imu_mid360.yaml")
    if external is not None:
        scale = external.get("imuAccelerationScale")
        if not finite_number(scale) or abs(float(scale) - 1.0) > 1.0e-9:
            errors.append(
                "params_imu_external.yaml: SI sensor profile must use "
                "imuAccelerationScale=1.0"
            )
        if external.get("imuOrientationSource") != "message":
            errors.append(
                "params_imu_external.yaml: calibrated attitude IMU must use "
                "imuOrientationSource=message"
            )
    if mid360 is not None:
        scale = mid360.get("imuAccelerationScale")
        if not finite_number(scale) or not 9.0 <= float(scale) <= 10.0:
            errors.append(
                "params_imu_mid360.yaml: raw acceleration in g requires an "
                "imuAccelerationScale between 9.0 and 10.0"
            )
        if mid360.get("imuRPYWeight") != 0.0:
            errors.append(
                "params_imu_mid360.yaml: imuRPYWeight must remain 0.0 because "
                "the driver does not publish an attitude estimate"
            )
        if mid360.get("imuOrientationSource") != "mount":
            errors.append(
                "params_imu_mid360.yaml: driver attitude is unavailable, so "
                "imuOrientationSource must be mount"
            )
    if external is not None and mid360 is not None and (
        external.get("imuTopic") == mid360.get("imuTopic")
    ):
        errors.append("external and MID-360 IMU profiles must use distinct topics")
    return loaded


def validate_mount_profiles(config_dir: Path, errors: list[str]) -> dict:
    loaded = {}
    for name in MOUNT_PROFILES:
        try:
            params = load_parameters(config_dir / name)
        except ValueError as exc:
            errors.append(str(exc))
            continue
        loaded[name] = params
        validate_vector(errors, name, params, "baseToLidarRotation", 9)
        validate_vector(errors, name, params, "baseToLidarTranslation", 3)
        validate_rotation(errors, name, params, "baseToLidarRotation")
    return loaded


def validate_camera_config(
    config_dir: Path,
    lidar_configs: dict,
    imu_profiles: dict,
    errors: list[str],
):
    name = "params_camera.yaml"
    try:
        params = load_parameters(config_dir / name)
    except ValueError as exc:
        errors.append(str(exc))
        return

    for key in (
        "PROJECT_NAME", "image_topic", "imu_topic", "odom_topic",
        "point_cloud_topic", "camera_name", "model_type",
    ):
        if not isinstance(params.get(key), str) or not params[key].strip():
            errors.append(f"{name}: {key} must be a non-empty string")
    camera_model = str(params.get("model_type", "")).lower()
    if camera_model not in (
        "pinhole", "mei", "kannala_brandt", "scaramuzza",
    ):
        errors.append(f"{name}: model_type is not supported by CameraFactory")
    project_name = str(params.get("PROJECT_NAME", "")).strip().strip("/")
    if project_name and not re.fullmatch(
        r"[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*",
        project_name,
    ):
        errors.append(
            f"{name}: PROJECT_NAME contains an invalid topic namespace"
        )
    for key in ("image_topic", "imu_topic", "odom_topic", "point_cloud_topic"):
        if (
            isinstance(params.get(key), str)
            and params[key]
            and not params[key].startswith("/")
        ):
            errors.append(f"{name}: {key} must be an absolute topic")
    for key in ("image_width", "image_height", "max_cnt", "min_dist"):
        if not isinstance(params.get(key), int) or params[key] <= 0:
            errors.append(f"{name}: {key} must be a positive integer")
    if not isinstance(params.get("freq"), int) or params["freq"] < 0:
        errors.append(f"{name}: freq must be a non-negative integer")
    if not isinstance(params.get("lidar_skip"), int) or params["lidar_skip"] < 0:
        errors.append(f"{name}: lidar_skip must be a non-negative integer")
    for key in ("use_lidar", "use_lidar_odometry_prior",
                "align_camera_lidar_estimation", "show_track",
                "equalize", "fisheye", "loop_closure", "debug_image",
                "estimate_td", "rolling_shutter"):
        if params.get(key) not in (0, 1):
            errors.append(f"{name}: {key} must be 0 or 1")
    if params.get("align_camera_lidar_estimation") == 1 and params.get("use_lidar") != 1:
        errors.append(f"{name}: align_camera_lidar_estimation requires use_lidar=1")
    if params.get("estimate_extrinsic") not in (0, 1, 2):
        errors.append(f"{name}: estimate_extrinsic must be 0, 1 or 2")
    for key in ("F_threshold", "max_solver_time", "keyframe_parallax",
                "acc_n", "acc_w", "gyr_n", "gyr_w", "g_norm",
                "imuAccelerationScale", "loop_sync_tolerance"):
        if not finite_number(params.get(key)) or float(params[key]) <= 0.0:
            errors.append(f"{name}: {key} must be finite and greater than 0")
    for key in ("skip_time", "skip_dist", "rolling_shutter_tr"):
        if not finite_number(params.get(key)) or float(params[key]) < 0.0:
            errors.append(f"{name}: {key} must be finite and non-negative")
    if not finite_number(params.get("td")):
        errors.append(f"{name}: td must be finite")
    rolling_shutter_time = params.get("rolling_shutter_tr")
    if params.get("rolling_shutter") == 1 and (
        not finite_number(rolling_shutter_time)
        or float(rolling_shutter_time) <= 0.0
    ):
        errors.append(
            f"{name}: rolling_shutter_tr must be positive when rolling_shutter=1"
        )
    match_image_scale = params.get("match_image_scale")
    if (
        not finite_number(match_image_scale)
        or not 0.0 < float(match_image_scale) <= 1.0
    ):
        errors.append(f"{name}: match_image_scale must be in (0, 1]")
    if not isinstance(params.get("max_num_iterations"), int) or params["max_num_iterations"] <= 0:
        errors.append(f"{name}: max_num_iterations must be a positive integer")
    if not isinstance(params.get("loop_min_index_gap"), int) or params[
        "loop_min_index_gap"
    ] <= 0:
        errors.append(f"{name}: loop_min_index_gap must be a positive integer")
    primary_score = params.get("loop_primary_score_threshold")
    secondary_score = params.get("loop_secondary_score_threshold")
    if (
        not finite_number(primary_score)
        or not finite_number(secondary_score)
        or not 0.0 < float(secondary_score) <= float(primary_score) <= 1.0
    ):
        errors.append(
            f"{name}: loop scores must satisfy 0 < secondary <= primary <= 1"
        )
    validate_vector(errors, name, params, "extrinsicRotation", 9)
    validate_vector(errors, name, params, "extrinsicTranslation", 3)
    validate_rotation(errors, name, params, "extrinsicRotation")
    if camera_model == "pinhole":
        projection = params.get("projection_parameters")
        if not isinstance(projection, dict):
            errors.append(f"{name}: projection_parameters must be a mapping")
        else:
            for key in ("fx", "fy"):
                if (
                    not finite_number(projection.get(key))
                    or float(projection[key]) <= 0.0
                ):
                    errors.append(
                        f"{name}: projection_parameters.{key} must be positive"
                    )
            for key, limit_key in (
                ("cx", "image_width"), ("cy", "image_height")
            ):
                value = projection.get(key)
                limit = params.get(limit_key)
                if (
                    not finite_number(value)
                    or not isinstance(limit, int)
                    or not 0.0 <= float(value) < float(limit)
                ):
                    errors.append(
                        f"{name}: projection_parameters.{key} must lie inside the image"
                    )
        distortion = params.get("distortion_parameters")
        if not isinstance(distortion, dict):
            errors.append(f"{name}: distortion_parameters must be a mapping")
        elif not all(
            finite_number(distortion.get(key)) for key in ("k1", "k2", "p1", "p2")
        ):
            errors.append(f"{name}: pinhole distortion coefficients must be finite")
    for key in ("lidar_to_cam_tx", "lidar_to_cam_ty", "lidar_to_cam_tz",
                "lidar_to_cam_rx", "lidar_to_cam_ry", "lidar_to_cam_rz"):
        if not finite_number(params.get(key)):
            errors.append(f"{name}: {key} must be finite")

    if lidar_configs:
        lidar = next(iter(lidar_configs.values()))
        if params.get("imu_topic") != lidar.get("imuTopic"):
            errors.append(f"{name}: imu_topic must match LIS imuTopic")
        if normalize_topic(params.get("odom_topic", "")) != normalize_topic(
            lidar.get("odomTopic", "")
        ):
            errors.append(f"{name}: odom_topic must match LIS odomTopic")

    external_imu = imu_profiles.get("params_imu_external.yaml")
    if external_imu is not None:
        for key in IMU_PROFILE_VIS_KEYS:
            if params.get(key) != external_imu.get(key):
                errors.append(
                    f"{name}: {key} must match params_imu_external.yaml to "
                    "preserve the default external-IMU path"
                )

    for key in ("vocabulary_file", "brief_pattern_file", "vins_config_file"):
        value = params.get(key)
        if not isinstance(value, str) or not value:
            errors.append(
                f"{name}: {key} must be a non-empty absolute or package-relative path"
            )
            continue
        asset = resolve_asset(config_dir, value)
        if not asset.is_file():
            errors.append(f"{name}: {key} does not exist: {asset}")

    if params.get("fisheye") == 1:
        fisheye_mask = params.get("fisheye_mask")
        if not isinstance(fisheye_mask, str) or not fisheye_mask:
            errors.append(f"{name}: fisheye_mask is required when fisheye=1")
        elif not resolve_asset(config_dir, fisheye_mask).is_file():
            errors.append(f"{name}: fisheye_mask does not exist")

    pattern_path = resolve_asset(
        config_dir, str(params.get("brief_pattern_file", ""))
    )
    if pattern_path.is_file():
        try:
            pattern = load_parameters(pattern_path)
            for key in ("x1", "y1", "x2", "y2"):
                if not isinstance(pattern.get(key), list) or len(pattern[key]) != 256:
                    errors.append(
                        f"{pattern_path.name}: {key} must contain exactly 256 integers"
                    )
        except ValueError as exc:
            errors.append(str(exc))


def source_config_dir() -> Path:
    candidate = Path(__file__).resolve().parent.parent / "config"
    if candidate.is_dir():
        return candidate
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("lvi_sam")) / "config"
    except Exception as exc:
        raise RuntimeError("cannot locate the lvi_sam config directory") from exc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", type=Path, default=None)
    parser.add_argument(
        "--lidar-only",
        action="store_true",
        help="skip VIS configuration and asset validation",
    )
    args = parser.parse_args()
    config_dir = args.config_dir.resolve() if args.config_dir else source_config_dir()

    errors: list[str] = []
    lidar_configs = validate_lidar_configs(config_dir, errors)
    imu_profiles = validate_imu_profiles(config_dir, errors)
    validate_mount_profiles(config_dir, errors)
    if not args.lidar_only:
        validate_camera_config(config_dir, lidar_configs, imu_profiles, errors)
    if errors:
        print("LVI-SAM configuration validation failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    print(f"LVI-SAM configuration validation passed: {config_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
