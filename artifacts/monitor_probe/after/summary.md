# monitor probe (hold_pose)

- state: p50 9.5 ms / p95 23.9 ms / max 23.9 ms (17 samples, 0 errors)
- frames: p50 32.8 ms / p95 62.7 ms / max 62.7 ms (5 samples, 0 errors)
- health: {"ms": 15.735699998913333, "delay_ms": 0.0, "status": 200, "json": {"gateway": "ok", "worker": "alive", "simulation_ready": true, "startup_lost_jobs": 0}, "error": null, "t": 5.016}
- cancel: {"request": {"ms": 69.03470000543166, "delay_ms": 0.0, "status": 202, "json": {"job_id": "job_8a5031a0199d42408a355a6ba8d00027", "kind": "skill", "status": "canceling", "simulation_id": "sim_6f320a0f02d74f82a98f420960daa3cb", "task_id": null, "skill": "hold_pose", "worker_job_id": "job_47fa8816cbfc48028fb32a7369caaaaf", "parent_job_id": null, "progress": 0.3, "cancel_requested": true, "created_at": "2026-11-07T12:05:20.141426+00:00", "updated_at": "2026-11-07T12:05:28.211388+00:00"}, "error": null, "t": 8.069}, "settled_ms": 65.3, "final_status": "canceled", "error": null}
- frame sim_step: [0, 315, 653, 982, 1331]
- events endpoint: {'status': 200, 'count': 8, 'types': ['created', 'created', 'started', 'progress', 'progress', 'progress', 'progress', 'canceled'], 'seq': [245, 246, 247, 248, 249, 250, 251, 252]}
- verdict: {"state_p95_under_200ms": true, "state_samples_present": true, "frame_max_under_1000ms": true, "frames_increasing": true, "cancel_ack_under_1000ms": true, "cancel_final_canceled": true, "health_under_100ms": true, "all_pass": true}