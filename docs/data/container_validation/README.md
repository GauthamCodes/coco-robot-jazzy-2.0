# Container validation evidence (2026-09-25)

Machine-readable evidence for `docs/DOCKER.md`. Every JSON file here is
**generated** by `scripts/container/condense_evidence.py` from the raw
runs listed in `index.json`; nothing is typed by hand. The raw runs (build
logs, container logs, ROS logs, JUnit XML — several hundred MB) stay
outside the repository, at the `root` named in `index.json`, on the
machine that produced them.

```bash
scripts/container/condense_evidence.py docs/data/container_validation/index.json docs/data/container_validation/
```

| File | What it holds |
|---|---|
| `environment.json` | host and Docker facts at the time of the runs |
| `builds.json` | every image build: rc, wall time, image id and size, per-step timings over 1 s, which commit (and infra commit) |
| `tests.json` | every test run, host and container: totals, per-package counts and times, every non-passing test with its message |
| `platform.json` | every appliance boot: time to healthy, `/healthz`, controller states, `cmd_vel` publisher set, the graph probe (RTF, topic rates, TF), steady CPU/RAM, the coco.v1 WebSocket probe, the Nav2 probe |
| `boot_race.json` | the controller activation race: boots by image and by render/activation order, before and after the fix |
| `determinism.json` | a `--no-cache` rebuild vs the cached build of the same commit: layer by layer, then file contents |
| `failures.json` | the reproduction bundles `validate.sh` wrote for a genuinely failing image, and the live capture of the activation-race boot |
| `regression.json` | the colour-fetch regression, one fresh container per run; `void` names a run that spanned a host suspend |
| `episodes.json` | an episode executed from its seed: manifest hash, host/image manifest identity, the task view the robot got, the EpisodeResult and its reproducibility check |
| `index.json` | the run list: label, kind, raw path, and a note on anything unusual |

Conventions, the repo's own: a number is here only if a run produced it;
a run that failed is listed as failing; a run stopped by hand says so in
its note. Wall-clock timings are wall-clock (`_wall_s`/`seconds`); the
simulator's time is labelled as such.
