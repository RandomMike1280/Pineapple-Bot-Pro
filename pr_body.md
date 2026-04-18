## Summary
- Phase out precision speed floor linearly inside settling band (20mm to target), preventing the 18mm/s minimum floor from forcing overshoot near target
- Increase predictive brake safety margin (1.5 -> 2.5) for earlier braking
- Strengthen translation brake: more frames, higher duty, lower trigger speed
- Reduce PRECISION_MIN_SPEED_LIMIT from 18 to 10 mm/s

## Changes
- `src/main.h`: Reduced precision min speed, increased predictive brake safety, strengthened translation brake parameters
- `lib/MotionQueue/motion_queue.cpp`: Phased out precision floor near target with linear fade in settling band; updated debug diagnostics to match

## Test plan
- [ ] Test robot approaching a waypoint at normal speed — observe no overshoot
- [ ] Test robot at slow speed approaching target — verify smooth deceleration
- [ ] Monitor `[WIG]` serial log to verify speed_scale drops naturally near target
