(define (problem emergency-template)
(:domain emergency)
(:objects 
  drone1 drone2 drone3 - drone
  turtlebot1 turtlebot2 - turtlebot
  w1 w2 w3 w4 w6 w7 w8 w9 w10 w11 w12 - waypoint
  w1a w2a w3a w4a w6a w7a w8a w9a w10a w11a w12a - airwaypoint
  box1 box2 box3 box4 - box
  person1 person2 person3 person4 - person)

; I acciendaly made 4/5 same waypoint, so removed 5
(:init 

  (= (move-duration w1 w2) 1)
  (= (move-duration w1 w3) 1)
  (= (move-duration w1 w4) 1)
  (= (move-duration w1 w6) 1)

  (= (move-duration w2 w1) 1)
  (= (move-duration w2 w3) 1)

  (= (move-duration w3 w2) 1)
  (= (move-duration w3 w1) 1)
  (= (move-duration w3 w4) 1)
  (= (move-duration w3 w9) 1)
  (= (move-duration w3 w12) 1)

  (= (move-duration w4 w1) 1)
  (= (move-duration w4 w3) 1)
  (= (move-duration w4 w9) 1)
  (= (move-duration w4 w8) 1)
  (= (move-duration w4 w6) 1)

  (= (move-duration w6 w1) 1)
  (= (move-duration w6 w4) 1)
  (= (move-duration w6 w8) 1)
  (= (move-duration w6 w7) 1)

  (= (move-duration w7 w6) 1)
  (= (move-duration w7 w8) 1)
  (= (move-duration w7 w10) 1)

  (= (move-duration w8 w6) 1)
  (= (move-duration w8 w4) 1)
  (= (move-duration w8 w9) 1)
  (= (move-duration w8 w11) 1)
  (= (move-duration w8 w10) 1)
  (= (move-duration w8 w7) 1)

  (= (move-duration w9 w4) 1)
  (= (move-duration w9 w3) 1)
  (= (move-duration w9 w12) 1)
  (= (move-duration w9 w11) 1)
  (= (move-duration w9 w8) 1)

  (= (move-duration w10 w7) 1)
  (= (move-duration w10 w8) 1)
  (= (move-duration w10 w11) 1)
  (= (move-duration w10 w12) 1)

  (= (move-duration w11 w9) 1)
  (= (move-duration w11 w12) 1)
  (= (move-duration w11 w10) 1)
  (= (move-duration w11 w8) 1)

  (= (move-duration w12 w3) 1)
  (= (move-duration w12 w9) 1)
  (= (move-duration w12 w11) 1)
  (= (move-duration w12 w10) 1)

  (= (move-duration w1a w2a) 1)
  (= (move-duration w1a w3a) 1)
  (= (move-duration w1a w4a) 1)
  (= (move-duration w1a w6a) 1)

  (= (move-duration w2a w1a) 1)
  (= (move-duration w2a w3a) 1)

  (= (move-duration w3a w2a) 1)
  (= (move-duration w3a w1a) 1)
  (= (move-duration w3a w4a) 1)
  (= (move-duration w3a w9a) 1)
  (= (move-duration w3a w12a) 1)

  (= (move-duration w4a w1a) 1)
  (= (move-duration w4a w3a) 1)
  (= (move-duration w4a w9a) 1)
  (= (move-duration w4a w8a) 1)
  (= (move-duration w4a w6a) 1)

  (= (move-duration w6a w1a) 1)
  (= (move-duration w6a w4a) 1)
  (= (move-duration w6a w8a) 1)
  (= (move-duration w6a w7a) 1)

  (= (move-duration w7a w6a) 1)
  (= (move-duration w7a w8a) 1)
  (= (move-duration w7a w10a) 1)

  (= (move-duration w8a w6a) 1)
  (= (move-duration w8a w4a) 1)
  (= (move-duration w8a w9a) 1)
  (= (move-duration w8a w11a) 1)
  (= (move-duration w8a w10a) 1)
  (= (move-duration w8a w7a) 1)

  (= (move-duration w9a w4a) 1)
  (= (move-duration w9a w3a) 1)
  (= (move-duration w9a w12a) 1)
  (= (move-duration w9a w11a) 1)
  (= (move-duration w9a w8a) 1)

  (= (move-duration w10a w7a) 1)
  (= (move-duration w10a w8a) 1)
  (= (move-duration w10a w11a) 1)
  (= (move-duration w10a w12a) 1)

  (= (move-duration w11a w9a) 1)
  (= (move-duration w11a w12a) 1)
  (= (move-duration w11a w10a) 1)
  (= (move-duration w11a w8a) 1)

  (= (move-duration w12a w3a) 1)
  (= (move-duration w12a w9a) 1)
  (= (move-duration w12a w11a) 1)
  (= (move-duration w12a w10a) 1)


  (over w1a w1)
  (over w2a w2)
  (over w3a w3)
  (over w4a w4)
  (over w6a w6)
  (over w7a w7)
  (over w8a w8)
  (over w9a w9)
  (over w10a w10)
  (over w11a w11)
  (over w12a w12)

  (free box1)
  (free box2)
  (free box3)
  (free box4)

  (at box1 w7)
  (at box2 w10)
  (at box3 w6)
  (at box4 w12)

  (empty turtlebot1)
  (empty turtlebot2)
  (empty drone1)
  (empty drone2)
  (empty drone3)

  (at drone1 w1a)
  (occupied w1a)

  (at drone2 w2a)
  (occupied w2a)

  (at drone3 w3a)
  (occupied w3a)

  (at turtlebot1 w1)
  (occupied w1)

  (at turtlebot2 w2)
  (occupied w2)

  (empty w3)
  (empty w4)
  (empty w4a)
  (empty w6)
  (empty w6a)
  (empty w7)
  (empty w7a)
  (empty w8)
  (empty w8a)
  (empty w9)
  (empty w9a)
  (empty w10)
  (empty w10a)
  (empty w11)
  (empty w11a)
  (empty w12)
  (empty w12a)
  
  (at person1 w8)
  (at person2 w11)
  (at person3 w9)
  (at person4 w4))

(:goal (and (handled person1)
            (handled person2)
            (handled person3)
            (handled person4))))

