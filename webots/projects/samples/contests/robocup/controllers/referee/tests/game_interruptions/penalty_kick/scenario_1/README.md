# Penalty Kick - scenario 1

## What is tested

- When a ball holding foul is committed inside the penalty area of the defender it ends with a free kick
- The ball is placed on the penalty mark at phase 1 of the game interruption
- The goalkeeper is allowed to stay on his goal line
- Robots in invalid positions are penalized
  - Only one robot of the striker team is allowed to stay near the ball
  - The goalkeeper of the defending team is allowed to stay on his goal line
  - No player other than the goalkeeper is allowed to be between the penalty mark and the goal line (along x-axis)
  - Robots further away from the ball are penalized
- When moved away from the ball, robots are not spawned on top of each other

## Setup

- The ball is in play
- Team `RED` has 4 robots
- Team `BLUE` has 3 robots

## Description

1. Robots `RED 2` and `RED 3` perform collective ball holding in `RED` penalty area
2. A penalty kick is awarded to team `BLUE`
3. During phase 0 of the penalty kick, `RED 2` is near the penalty mark and `BLUE 3` right behind him
4. At beginning of phase 1:
   - Ball is placed on the penalty mark
   - `RED 2` and `BLUE 3` are moved away from the ball and they do not collide with each other
5. During phase 1, the robots take the following positions
   - `RED 1` : on goal line from team red -> `VALID` (goalkeeper)
   - `RED 2` : on goal line from team red -> `INVALID`
   - `RED 3` : away from ball between goal line and penalty mark -> `INVALID`
   - `RED 4` : away from ball behind penalty mark -> `VALID`
   - `BLUE 1` : on goal line from team red -> `INVALID`
   - `BLUE 2` : near the ball, behind penalty mark -> `VALID`
   - `BLUE 3` : near the ball, behind penalty mark-> `INVALID` (`BLUE 2` is already striker)
6. At beginning of phase 2, robots are penalized and removed based on the description phase 5
