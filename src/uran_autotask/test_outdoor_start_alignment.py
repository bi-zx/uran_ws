from uran_autotask.outdoor import OutdoorExecutionPoint, OutdoorMissionPlan, OutdoorStartAligner


def _mission():
    return OutdoorMissionPlan(
        task_id='task_start_alignment',
        task_type='mission_planner_route',
        scene_name='demo',
        planner_result_id='plan_demo',
        robot_id='dog_1',
        map_name='',
        frame_id='map',
        execution_points=[
            OutdoorExecutionPoint(seq=0, kind='start', x=0.0, y=0.0),
            OutdoorExecutionPoint(seq=1, kind='transit', x=5.0, y=0.0),
            OutdoorExecutionPoint(seq=2, kind='inspection', x=10.0, y=1.0),
        ],
    )


def test_start_aligner_skips_explicit_start_when_already_close():
    mission = _mission()
    aligner = OutdoorStartAligner({
        'aligned_tolerance_m': 1.0,
        'skip_explicit_start': True,
    })

    result = aligner.evaluate_and_apply(
        mission=mission,
        current_pose={
            'x': 0.5,
            'y': 0.0,
            'z': 0.0,
            'frame_id': 'map',
            'timestamp_ns': 1_000_000_000,
        },
        now_ns=1_100_000_000,
    )

    assert result['status'] == 'start_skipped'
    assert result['action'] == 'skip_start'
    assert result['next_current_waypoint_index'] == 0
    assert mission.execution_points[1].x == 5.0


def test_start_aligner_returns_startup_correction_when_start_offset_is_allowed():
    mission = _mission()
    aligner = OutdoorStartAligner({
        'aligned_tolerance_m': 1.0,
        'skip_start_tolerance_m': 3.0,
        'allow_start_correction': True,
        'max_start_correction_m': 10.0,
        'skip_explicit_start': True,
    })

    result = aligner.evaluate_and_apply(
        mission=mission,
        current_pose={
            'x': 4.0,
            'y': -2.0,
            'z': 0.0,
            'frame_id': 'map',
            'timestamp_ns': 1_000_000_000,
        },
        now_ns=1_100_000_000,
    )

    assert result['status'] == 'start_correction_applied'
    assert result['action'] == 'apply_start_correction_and_skip_start'
    assert result['next_current_waypoint_index'] == 0
    assert result['goal_correction']['dx'] == 4.0
    assert result['goal_correction']['dy'] == -2.0
    assert mission.execution_points[0].x == 0.0
    assert mission.execution_points[0].y == 0.0
    assert mission.execution_points[1].x == 5.0
    assert mission.execution_points[1].y == 0.0


def test_start_aligner_rejects_large_start_offset():
    mission = _mission()
    aligner = OutdoorStartAligner({
        'allow_start_correction': True,
        'max_start_correction_m': 10.0,
    })

    result = aligner.evaluate_and_apply(
        mission=mission,
        current_pose={
            'x': 30.0,
            'y': 0.0,
            'z': 0.0,
            'frame_id': 'map',
            'timestamp_ns': 1_000_000_000,
        },
        now_ns=1_100_000_000,
    )

    assert result['status'] == 'failed'
    assert result['action'] == 'reject'
    assert mission.execution_points[0].x == 0.0


def test_start_aligner_waits_for_fresh_map_pose():
    mission = _mission()
    aligner = OutdoorStartAligner({'max_pose_age_s': 3.0})

    result = aligner.evaluate_and_apply(
        mission=mission,
        current_pose={'x': 0.0, 'y': 0.0, 'timestamp_ns': 1_000_000_000},
        now_ns=5_000_000_000,
    )

    assert result['status'] == 'stale_pose'
    assert result['action'] == 'wait'
