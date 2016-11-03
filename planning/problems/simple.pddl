(define (problem emergency-template)
(:domain emergency)
(:objects 
  drone1 - drone
  turtlebot1 - turtlebot
  w x y - waypoint
  w2 x2 y2 - airwaypoint
  box1 - box
  person1 - person)

(:init 

  (= (move-duration w2 x2) 6)
  (= (move-duration x2 w2) 6)
  (= (move-duration x2 y2) 8)
  (= (move-duration y2 x2) 8)
  (= (move-duration y2 w2) 10)
  (= (move-duration w2 y2) 10)

  (= (move-duration w x) 3)
  (= (move-duration x w) 3)
  (= (move-duration x y) 4)
  (= (move-duration y x) 4)
  (= (move-duration y w) 5)
  (= (move-duration w y) 5)

  (over w2 w)
  (over x2 x)
  (over y2 y)

  (free box1)

  (at box1 w)

  (empty turtlebot1)
  (empty drone1)

  (at drone1 x2)
  (occupied x2)

  (at turtlebot1 x)
  (occupied x)

  (empty w2)
  (empty y2)
  (empty w)
  (empty y)
  
  (at person1 y))

(:goal (and (handled person1))))

;(:goal (and (at drone1 x2) (at turtlebot1 w) (carrying turtlebot1 box1))))