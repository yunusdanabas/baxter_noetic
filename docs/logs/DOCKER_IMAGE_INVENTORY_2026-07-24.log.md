---
step: disk_cleanup
title: Docker image inventory before reclaim
agent_date: 2026-07-24
status: logged
previous_steps: [N07, N08, N09, N09-5, N10, N12, PX2, PX3]
---

# Docker image inventory (2026-07-24)

Local snapshot before reclaiming Docker Desktop storage.  
Mirror: `/home/yunusdanabas/baxter_ros2_jazzy/docker/local_image_inventory.md`

## Findings

- Docker Desktop UI showed **28.93 GB / 19–20 images**; CLI on `desktop-linux` matched (~**28.95G** images + **13.81G** build cache).
- Host file `~/.docker/desktop/vms/0/data/Docker.raw` still held **~41G allocated** on disk (182G virtual max).
- A second engine exists: host `dockerd` (`unix:///var/run/docker.sock`) with older `baxter-noetic:audit` / `n07` (different image IDs than Desktop).
- Host free space at snapshot: **13G** (93% used).

## Desktop images — Baxter Noetic tags

All built from this workspace via `baxter_simulator/Dockerfile` (see `logs/N07_docker_setup.log.md`). Working dir: `~/baxter_noetic_ws/src/baxter_noetic`.

| Tag | Image ID | Size | Created | Step / notes |
|---|---|---:|---|---|
| cleanup | `048c6c54d5fd` | 5.59GB | 2026-07-16 13:47 | Post-hygiene rebuild |
| px3 | `f152f2aca1e5` | 5.59GB | 2026-07-16 12:12 | PX3 |
| n07 | `2b51205e69e9` | 5.59GB | 2026-07-16 12:12 | **Canonical** — `docker-compose.yml` default |
| px2 | `e58258efec80` | 5.59GB | 2026-07-16 11:53 | PX2 |
| n12 | `b02449dfdfe5` | 5.59GB | 2026-07-16 10:53 | N12 |
| n10 | `f8dcd123fadd` | 5.59GB | 2026-07-15 19:22 | N10 |
| n09-5 | `2f6a5dc35e80` | 5.59GB | 2026-07-15 18:55 | N09.5 |
| ci | `52874186d88a` | 5.59GB | 2026-07-15 17:17 | CI workflow tag |

Reported sizes share layers; total unique cost is far below 8 × 5.59G.

## Other Desktop images (not from this repo)

| Image | ID | Size | Rebuild |
|---|---|---:|---|
| `baxter_ros2_jazzy-devcontainer:latest` | `c8b9526bca8c` | 6.59GB | ROS 2 Jazzy `.devcontainer` |
| `ros:foxy-ros1-bridge-focal` | `b22bd431548c` | 3.39GB | `docker pull` |
| `mcp/ros2` (`<none>`) | `ec6572387d80` | 3.89GB | Docker MCP catalog |
| `mcp/markdownify` (`<none>`) | `afa1ab8e1563` | 1.55GB | Docker MCP catalog |
| `ruby:3.3.5` | `bceec7582aaa` | 1.47GB | `docker pull` |
| `microros/micro-ros-agent:jazzy` | `1d13bfebcd8a` | 741MB | `docker pull` |
| `mcp/fetch` (`<none>`) | `d9907377c03d` | 433MB | Docker MCP catalog |
| `mcp/context7` (`<none>`) | `1174e6a29634` | 425MB | Docker MCP catalog |
| `mcp/arxiv-mcp-server` (`<none>`) | `6dc6bba6dfed` | 339MB | Docker MCP catalog |
| `mcp/sequentialthinking` (`<none>`) | `cd3174b2ecf3` | 236MB | Docker MCP catalog |
| `mcp/memory` (`<none>`) | `db0c2db07a44` | 233MB | Docker MCP catalog |
| `alpine:latest` | `28bd5fe8b56d` | 13MB | `docker pull` |

Build cache on Desktop at snapshot: **13.81G** (58 entries).

## Host dockerd images (separate)

| Image | ID | Size | Created |
|---|---|---:|---|
| `baxter-noetic:audit` | `91786a3c88f6` | 4.25GB | 2026-07-16 01:28 |
| `baxter-noetic:n07` | `92c51585e10e` | 4.25GB | 2026-07-14 17:53 |
| `justincormack/nsenter1:latest` | `c81481184b1b` | 101kB | diag |

Desktop `n07` (`2b51205e69e9`) ≠ host `n07` (`92c51585e10e`).

## Rebuild later (Noetic)

Preferred single tag after cleanup:

```bash
cd ~/baxter_noetic_ws/src/baxter_noetic
docker context use desktop-linux
docker build -f baxter_simulator/Dockerfile -t baxter-noetic:n07 .
```

Optional step tags (only if needed):

```bash
docker build -f baxter_simulator/Dockerfile -t baxter-noetic:ci .
# likewise: n09-5 n10 n12 px2 px3 cleanup
```

Compose stack (expects `baxter-noetic:n07`):

```bash
cd ~/baxter_noetic_ws/src/baxter_noetic
docker compose up
```

## Artifacts

- This log: `logs/DOCKER_IMAGE_INVENTORY_2026-07-24.log.md`
- ROS 2 mirror: `~/baxter_ros2_jazzy/docker/local_image_inventory.md`
- Canonical build recipe: `logs/N07_docker_setup.log.md`, `baxter_simulator/Dockerfile`, `docker-compose.yml`
