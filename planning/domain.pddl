(define (domain emergency)

(:requirements :strips :typing :equality :action-costs)

(:types 
  waypoint - object 
  depot - waypoint
  meeting-place - depot
  ;agent is supertype of turtlebot and drone
  agent - object
  turtlebot drone - agent
  person - object
  box - object)

(:predicates
  (at ?agent - agent ?waypoint - waypoint)
  (handled ?person - person)
  (free ?box - box)
  (empty ?agent - agent)
  (carrying ?agent - agent ?box - box))

;move-durations will be different for turtlebot/drone, hence the agent parameter
;additionally drone cannot move to all places
(:functions (move-duration ?agent - agent ?from ?to - waypoint) - number
            (total-cost) - number)

(:action move
  :parameters (?agent - agent ?from ?to - waypoint)
  :precondition (at ?agent ?from)
  :effect (and (increase (total-cost) (move-duration ?agent ?from ?to))
               (not (at ?agent ?from))
               (at ?agent ?to)))

;only drone can do pick-up, from a known waypoint
(:action pick-up
  :parameters (?drone - drone ?box - box ?waypoint - waypoint)
  :precondition (and (at ?drone ?waypoint)
                  (at ?box ?waypoint)
                  (empty ?drone)
                  (free ?box))
  :effect (and (carrying ?drone ?box)
               (not (empty ?drone))
               (not (free ?box))))

(:action hand-over
  :parameters (?drone - drone ?turtlebot - turtlebot ?box - box ?waypoint - meeting-place)
  :precondition (and (at ?drone ?waypoint)
                     (at ?turtlebot ?waypoint)
                     (carrying ?drone ?box)
                     (empty ?turtlebot))
  :effect (and (not (carrying ?drone ?box))
               (carrying ?turtlebot ?box)
               (empty ?drone)
               (not (empty ?turtlebot))))

(:action deliver
  :parameters (?turtlebot - turtlebot ?box - box ?waypoint - waypoint ?person - person)
  :precondition (and (carrying ?turtlebot ?box)
                     (at ?turtlebot ?waypoint)
                     (at ?person ?waypoint))
  :effect (and (not (carrying ?turtlebot ?box))
               (empty ?turtlebot)
               (not (free ?box))
               (handled ?person))))
