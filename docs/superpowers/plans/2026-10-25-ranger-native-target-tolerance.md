# Ranger native target tolerance

Implement the requested Ranger approach change in the existing workspace, preserving its pending work. Go2 retains its current approach strategy.

Ranger sends the detected object position, with current yaw, to Nav2. Remove Ranger's circular standoff selection and distance-triggered action cancellation. Reuse the earlier VLM_Nav native tolerance split: Navfn `GridBased.tolerance=0.76` and position goal checker `xy_goal_tolerance=0.05`, giving a nominal 0.81 m approach range. Keep viewpoint navigation accurate. A native successful action plus confirmed stop and measured target distance are required; cooperative acceptance remains bounded by its 1 m contract.

1. Add failing tests in `test_robot_local_exploration.py` and `test_private_c.py`: direct target requests, occupied target with valid tolerated endpoint, no distance cancellation, native failure, endpoint outside budget, stop/cancel/TF failures. Update adapter coverage for Ranger's legacy approach entry.
2. Change `go2_exploration.py` to use Ranger direct targets in cooperative and local modes, validate actual planner/controller parameters and endpoint, retain stop confirmation and final distance verification. Make Ranger adapter/fake approach requests return the target reference rather than candidate points; retain the shared Go2 candidate implementation.
3. Document native tolerances in `nav2_ranger_local.yaml`, update current README/runbook guidance, and run the focused Ranger/cooperative/adapter regression suites using `scripts/test_ranger_local.sh`. Review the final diff without committing unrelated workspace changes.

Review focus: target on an obstacle must not be rejected solely by target-footprint tests; invalid/empty paths and stale costmaps still fail; a Nav2 success outside the measured distance contract is not success; cancellation/late replies cannot start new motion; Go2 and observation-point selection remain covered by regressions.

## Validation

Focused regression: 287 tests passed across the Ranger approach/adapter, private and independent C, cooperative preparation, initial observation, generated candidates, Go2 exploration, navigation gate, launch, arrival, geometry and pipeline suites. Independent reviewer ran the three modified approach/adapter suites: 87 passed, no actionable finding.

Expanded run: 642 passed, 26 failed, 1 skipped. The task-related failure `test_cooperative_preparation.py::test_last_digest_wait_rechecks_motion_prerequisites_before_approach[footprint]` used the removed circular-candidate check; it now changes actual endpoint footprint availability and passes in the focused run. The other 25 failures were not changed by this task:

- `test_dynamic_costmap_config.py`: `test_effective_cooperative_costmaps_keep_dynamic_layers_and_other_parameters`, `test_actual_navigation_launch_applies_optional_overlay_last`, `test_isolated_acceptance_launch_has_only_costmaps_and_lifecycle_manager`, `test_no_verification_flags_are_set_by_static_costmap_profile`, `test_static_collection_dry_run_contains_only_read_commands`, `test_native_costmap_marking_and_clearing_with_synthetic_inputs_only[sigint,lifecycle_shutdown]`. Missing Go2-specific module/launch/script and verification-flag mismatch.
- `test_field_preparation.py`: `test_empty_resources_allow_independent_exploration_but_not_cooperative_live`, `test_offline_single_robot_config_does_not_require_cooperative_certification` (Go2 machine-specific map/config); `test_global_goal_age_config_must_be_positive_finite[0.0,-1.0,nan,inf,True,None]` (expected `Go2` vs actual `go2`); `test_readonly_observer_has_no_velocity_publisher_and_adapter_rejects_motion` (ROS observer setup).
- `test_go2_adapter.py::test_observation_only_node_cannot_enable_old_autonomy`, `test_robot_adapters.py::test_ranger_observer_keeps_own_name_under_agent_global_remap`, `test_robot_adapters.py::test_aligned_depth_observer_processes_disabled_input_without_autonomy[False,True]`, `test_ros_smoke.py::test_dual_fake_ros_mission_and_topics`: ROS context initialization failures in the combined run. The Ranger observer tests pass in the focused run.
- `test_go2_executing_path.py::test_execution_path_timeout_logs_receipt_age_once[go2,ranger]` and `test_go2_health_diagnostics.py::test_localization_loss_logs_failure_time_evidence_once_per_loss[stale,tf_error]`: expected messages reach stderr but are absent from `caplog` in the combined run.

No live robot motion, deployment or hardware/Nav2 arrival validation performed. Native planner tolerance is not an exact stopping radius; actual endpoint budget and final stopped distance remain mandatory.
