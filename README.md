# IndRob

This Project is the Can Crushing Line for those who want to collect beer cans but dont have the space. 

@UR20 contains files for the UR20 robot ~ both the class file with dh params and ply files for modelling
@UR3 contains files for the UR3 robot ~ both the class file with dh params and ply files for modelling ~ this was obtained from Peter Corke's toolbox. 

@MyWorld contains the function and ply files for loading in the world. 

Main is the function for running basic sequence using rmrc

collisionAvoidance is basic collision avoidance using random path when an obstacle is present

visualServoing is basic visual servoing using the UR3 which retreats when a marker comes too close



Explanation Notes:
GUI Creation:
- App designer 
- Drag and drop onto the GUI from component library in design view
- In code view:
  - startup function ~ not used 
  - ESTOP value change ~ toggle ~ call value through app.ESTOPBUTTON.value 
      if value = 1
      change the colour to red 
      change stop to true ~ a class specific variable 
      text ~ printf stopped. 
