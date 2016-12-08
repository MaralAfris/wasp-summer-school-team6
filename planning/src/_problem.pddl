(define (problem emergency-template)
(:domain emergency)
(:objects
  turtle1 - turtlebot
  drone1 - drone
  box1_wp drone1_wp person1_wp turtle1_wp - waypoint
  box1_wp_air drone1_wp_air person1_wp_air turtle1_wp_air - airwaypoint
  box1 - box
  person1 - person)

(:init
  (= (move-duration drone1_wp box1_wp) 2.90561519913)
  (= (move-duration box1_wp drone1_wp) 2.90561519913)
  (= (fly-duration drone1_wp_air box1_wp_air) 1.54336911948)
  (= (fly-duration box1_wp_air drone1_wp_air) 1.54336911948)
  (= (fly-carry-duration drone1_wp_air box1_wp_air) 6.43369119476)
  (= (fly-carry-duration box1_wp_air drone1_wp_air) 6.43369119476)
  (= (move-duration turtle1_wp box1_wp) 2.30299431604)
  (= (move-duration box1_wp turtle1_wp) 2.30299431604)
  (= (fly-duration turtle1_wp_air box1_wp_air) 1.18179658963)
  (= (fly-duration box1_wp_air turtle1_wp_air) 1.18179658963)
  (= (fly-carry-duration turtle1_wp_air box1_wp_air) 2.81796589627)
  (= (fly-carry-duration box1_wp_air turtle1_wp_air) 2.81796589627)
  (= (move-duration box1_wp person1_wp) 3.21335164821)
  (= (move-duration person1_wp box1_wp) 3.21335164821)
  (= (fly-duration box1_wp_air person1_wp_air) 1.72801098893)
  (= (fly-duration person1_wp_air box1_wp_air) 1.72801098893)
  (= (fly-carry-duration box1_wp_air person1_wp_air) 8.28010988928)
  (= (fly-carry-duration person1_wp_air box1_wp_air) 8.28010988928)

  (over box1_wp_air box1_wp)
  (over drone1_wp_air drone1_wp)
  (over person1_wp_air person1_wp)
  (over turtle1_wp_air turtle1_wp)

  (free box1)
  (at box1 box1_wp)

  (empty turtle1)
  (at turtle1 turtle1_wp)

  (empty drone1)
  (at drone1 drone1_wp_air)


  (empty person1_wp)
  (empty box1_wp)
  (empty box1_wp_air)
  (empty person1_wp_air)
  (empty drone1_wp)
  (empty turtle1_wp_air)

  (occupied drone1_wp_air)
  (occupied turtle1_wp)

  (at person1 person1_wp)
)

(:goal (and
            (handled person1)
)))
