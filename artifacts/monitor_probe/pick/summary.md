# monitor probe (pick)

- state: p50 8.4 ms / p95 37.0 ms / max 50.0 ms (30 samples, 0 errors)
- frames: p50 35.3 ms / p95 51.2 ms / max 51.2 ms (8 samples, 0 errors)
- health: {"ms": 1.6710000054445118, "delay_ms": 0.0, "status": 200, "json": {"gateway": "ok", "worker": "alive", "simulation_ready": true, "startup_lost_jobs": 0}, "error": null, "t": 10.002}
- cancel: {"request": null, "settled_ms": 5009.8, "final_status": "succeeded", "error": null}
- frame sim_step: [0, 300, 622, 863, 1108, 1351, 1592, 1908]
- events endpoint: {'status': 200, 'count': 15, 'types': ['created', 'created', 'started', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'progress', 'succeeded'], 'seq': [224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238]}
- verdict: {"state_p95_under_200ms": true, "state_samples_present": true, "frame_max_under_1000ms": true, "frames_increasing": true, "cancel_ack_under_1000ms": false, "cancel_final_canceled": true, "health_under_100ms": true, "all_pass": false}