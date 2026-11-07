# monitor probe (hold_pose)

- state: p50 2.5 ms / p95 17.4 ms / max 18.0 ms (85 samples, 0 errors)
- frames: p50 2.1 ms / p95 17.6 ms / max 47.4 ms (22 samples, 0 errors)
- health: {"ms": 12.445399999705842, "delay_ms": 0.0, "status": 200, "json": {"gateway": "ok", "worker": "alive", "simulation_ready": true, "startup_lost_jobs": 0}, "error": null, "t": 3.012}
- cancel: {"request": null, "settled_ms": 40002.3, "final_status": "running", "error": null}
- frame sim_step: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
- events endpoint: {'status': 200, 'count': 26, 'types': ['created', 'created', 'started', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress'], 'seq': [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]}
- verdict: {"state_p95_under_200ms": true, "state_samples_present": true, "frame_max_under_1000ms": true, "frames_increasing": true, "cancel_ack_under_1000ms": false, "cancel_final_canceled": true, "health_under_100ms": true, "all_pass": false}