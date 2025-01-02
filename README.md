# Krakel

## Setup

### Build

```bash
git clone https://github.com/moveit/krakel.git
cd krakel
docker compose build
```

### Run

```bash
docker compose up
docker compose exec -it krakel bash
```

### Launch

```bash
ros2 launch krakel_demos constrained_planning_demo.launch.py
```
