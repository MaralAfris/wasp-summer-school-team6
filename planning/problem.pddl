(define (problem emergency-template)
(:domain emergency)
(:objects 
  drone turtlebot - agent
  a b c - waypoint)

(:init 
  (= (move-duration turtlebot a a) 0)
  (= (move-duration drone a a) 0)
  (= (move-duration turtlebot a b) 15)
  (= (move-duration drone a b) 10)
  (= (move-duration turtlebot a c) 15)
  (= (move-duration drone a c) 10)

  (= (move-duration turtlebot b a) 15)
  (= (move-duration drone b a) 10)
  (= (move-duration turtlebot b b) 0)
  (= (move-duration drone b b) 0)
  (= (move-duration turtlebot b c) 15)
  (= (move-duration drone b c) 10)

  (= (move-duration turtlebot c a) 15)
  (= (move-duration drone c a) 10)
  (= (move-duration turtlebot c b) 15)
  (= (move-duration drone c b) 10)
  (= (move-duration turtlebot c c) 0)
  (= (move-duration drone c c) 0)

  (at drone a)
  (at turtlebot b)
)

(:goal (and (at drone c)
            (at turtlebot a))))

