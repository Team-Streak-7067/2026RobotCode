  **TASK**  |  **PRIORITY**  
------------|----------------
shooting on the move (teleop/auto)| ##
*pressing align in neutral zone aims for delivery* | *##*
*assisted trench crossing by aligning to apriltag* | *###*
mirror autos | ##
new auto path (neutral -> shoot x2) | ###

  **FIX**  | **PRIORITY**
-----------|-------------


### to test
- hub align shoot routine (rigorous test, try stopping at every command and see what happens) (tune deadline delay)
- shooter idling
- trench targeting distance
- ~~trench align~~

### changelog
- leds change color when aligned
- changed led target color to yellow
- driver intake bind now opens intake
- moved manual shooting, intake pid buttons, intake manual to operator controller
- restored vision update speed to default
- limited trench targeting range to 1.5m
- disabled moving when locked onto hub
- when trench is targeted, pressing aligns robot in front of trench april tag
- when pressing align in alliance zone, robot points to hub, waits for driver confirmation, then shoots
- shooter idles when in alliance zone and no shooter command is active