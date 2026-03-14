  **TASK**  |  **PRIORITY**  
------------|----------------
shooting on the move (teleop/auto)| ##
*pressing align in neutral zone aims for delivery* | *##*
*assisted trench crossing by aligning to apriltag* | *###*
reverse conveyor and intake commands | ##
mirror autos | ##

  **FIX**  | **PRIORITY**
-----------|-------------

### to test
- ~~trench align~~

### changelog
- driver intake bind now opens intake
- moved manual shooting, intake pid buttons, intake manual to operator controller
- restored vision update speed to default
- limited trench targeting range to 1.5m
- disabled moving when locked onto hub
- when trench is targeted, pressing aligns robot in front of trench april tag
- new improved auto path