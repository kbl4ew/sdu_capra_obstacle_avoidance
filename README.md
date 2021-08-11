# sdu_capra_obstacle_avoidance




# Evaluate badgr collisions on Pikachu

## Read Pikachu topics
We handle the communication between pikachu and the jetson through MQTT.
Add the Pikachu ROS topics to the roscore

```bash
put bash commands here
```

## Run badgr

Set the cost function weights of badgr to only account for collisions,
```bash
rosparam set /cost_weights "{'collision': 1.0, 'position': 0.0, 'position_sigmoid_center': 0.4, 'position_sigmoid_scale': 100., 'action_magnitude': 0.01, 'action_smooth': 0.0, 'bumpy': 0.0}"
```

and then start the policy
```bash
python scripts/eval.py configs/bumpy_collision_position.py
```

It will diplay an image with the candidate action sequences,predicted probabilities of collision, and the optimal action sequence for purely avoiding collisions.
