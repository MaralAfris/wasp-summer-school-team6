(define (domain emergency)
(:requirements :strips :typing :equality :durative-actions)
(:types waypoint agent - object)

(:predicates
  (at ?agent - agent ?waypoint - waypoint)
)

(:functions (move-duration ?agent - agent ?from ?to - waypoint) - number)

(:action move 
  :parameters (?agent - agent ?from ?to - waypoint)
  :duration (= ?duration (move-duration ?agent ?from ?to)) 
  :condition (at start (at ?agent ?from))
  :effect (and (at start (not (at ?agent ?from)))
               (at end (at ?agent ?to))))
)
