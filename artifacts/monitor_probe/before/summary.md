# monitor probe (hold_pose)

- state: p50 499.8 ms / p95 25906.6 ms / max 25906.6 ms (3 samples, 0 errors)
- frames: p50 26913.0 ms / p95 26913.0 ms / max 26913.0 ms (1 samples, 0 errors)
- health: {"ms": 16913.985300001514, "status": 200, "json": {"gateway": "ok", "worker": "alive", "simulation_ready": true, "startup_lost_jobs": 0}, "error": null, "t": 26.915}
- cancel: {"request": {"ms": 3.0508999989251606, "status": 202, "json": {"job_id": "job_206bbe20f20b4d88975de94a12ccdeca", "kind": "skill", "status": "succeeded", "simulation_id": "sim_15cd66323c1240dabf5cfbf3da58bb4c", "task_id": null, "skill": "hold_pose", "worker_job_id": "job_67dd5349d5494efc97517cd0ee96084d", "parent_job_id": null, "progress": 1.0, "cancel_requested": false, "created_at": "2026-11-07T11:30:45.342665+00:00", "updated_at": "2026-11-07T11:31:11.306296+00:00"}, "error": null, "t": 26.918}, "settled_ms": 25.9, "final_status": "succeeded", "error": null}
- frame sim_step: [4000]
- events endpoint: {'status': 404}
- verdict: {"state_p95_under_200ms": false, "frame_max_under_1000ms": false, "frames_increasing": false, "cancel_ack_under_1000ms": true, "health_under_100ms": false, "all_pass": false}