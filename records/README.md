# tune8 records for split main runs

When one controller's main experiment is split over several school PCs (`school/main8_chain.ps1 -Shards N -Index ...`),
the PCs that did not tune it need that controller's final tuning record. `main8_chain.ps1` looks for
`<controller>.record.json` and `<controller>.gate.json` here (and on the PC's Desktop, Downloads and USB drives).

Only files copied unchanged from the tuning PC (`fds8\results\arena\tuning\tune8\`) belong here. The chain
checks them before use: complete at 60, tuned on `configs/arena_tune8.json`, and a gate PASS on that exact record.
