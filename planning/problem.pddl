(define (problem emergency-template)
(:domain emergency)
(:objects 
  drone1 - drone
  turtlebot1 - turtlebot
  w x y z - waypoint
  a b c d - depot
  meeting-place - meeting-place
  box1 box2 box3 box4 - box
  person1 person2 person3 person4 - person)

(:init 
  (= (total-cost) 0)

  (= (move-duration drone1 meeting-place a) 1)
  (= (move-duration drone1 meeting-place b) 3)
  (= (move-duration drone1 meeting-place c) 3)
  (= (move-duration drone1 meeting-place d) 1)

  (= (move-duration drone1 a meeting-place) 1)
  (= (move-duration drone1 b meeting-place) 3)
  (= (move-duration drone1 c meeting-place) 3)
  (= (move-duration drone1 d meeting-place) 1)

  (= (move-duration turtlebot1 w meeting-place) 10)
  (= (move-duration turtlebot1 meeting-place w) 10)

  (= (move-duration turtlebot1 y meeting-place) 20)
  (= (move-duration turtlebot1 meeting-place y) 20)

  (= (move-duration turtlebot1 x meeting-place) 10)
  (= (move-duration turtlebot1 meeting-place x) 10)

  (= (move-duration turtlebot1 z meeting-place) 10)
  (= (move-duration turtlebot1 meeting-place z) 10)

  (free box1)
  (free box2)
  (free box3)
  (free box4)

  (at box1 a)
  (at box2 b)
  (at box3 c)
  (at box4 d)

  (empty turtlebot1)
  (empty drone1)

  (at drone1 meeting-place)
  (at turtlebot1 meeting-place)
  
  (at person1 w)
  (at person2 x)
  (at person3 y)
  (at person4 z))

(:goal (and (handled person1)
            (handled person2)
            (handled person3)
            (handled person4)))

(:metric minimize (total-cost)))
