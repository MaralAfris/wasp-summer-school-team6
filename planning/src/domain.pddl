(define (domain emergency)

(:requirements :strips :typing :equality :durative-actions)

(:types 
  waypoint - object
  airwaypoint - waypoint 
  ;agent is supertype of turtlebot and drone
  agent - object
  turtlebot drone - agent
  person - object
  box - object)

(:predicates
  (at ?agent - agent ?waypoint - waypoint)

  (over ?airwaypoint - airwaypoint ?waypoint - waypoint)

  (handled ?person - person)

  (free ?box - box)

  ; double state because of STRIPS
  (empty ?waypoint - waypoint)
  (occupied ?waypoint - waypoint)

  (empty ?agent - agent)
  (carrying ?agent - agent ?box - box))

(:functions (move-duration ?from ?to - waypoint) - number
            (fly-duration ?from ?to - waypoint) - number
            (fly-carry-duration ?from ?to - waypoint) - number)

;move turtlebot
(:durative-action move
  :parameters (?turtlebot - turtlebot ?from ?to - waypoint)
  :duration (= ?duration (move-duration ?from ?to))
  :condition (and (over all (empty ?to))
                  (at start (at ?turtlebot ?from)))
  :effect (and (at end (not (at ?turtlebot ?from)))
               (at end (at ?turtlebot ?to))
               (at end (not (occupied ?from)))
               (at end (empty ?from))
               (at start (not (empty ?to)))
               (at start (occupied ?to))))

;move drone with box (needs different name from 'move' to stop yahsp3 from segfaulting)
(:durative-action fly 
  :parameters (?drone - drone ?from ?to - waypoint ?box - box)
  :duration (= ?duration (fly-carry-duration ?from ?to))
  :condition (and (over all (carrying ?drone ?box))
                  (over all (empty ?to))
                  (at start (at ?drone ?from)))
  :effect (and (at end (not (at ?drone ?from)))
               (at end (at ?drone ?to))
               (at end (not (occupied ?from)))
               (at end (empty ?from))
               (at start (not (empty ?to)))
               (at start (occupied ?to))))

;move drone without box (needs different name from 'move' to stop yahsp3 from segfaulting)
(:durative-action fly
  :parameters (?drone - drone ?from ?to - waypoint)
  :duration (= ?duration (fly-duration ?from ?to))
  :condition (and (over all (empty ?drone))
                  (over all (empty ?to))
                  (at start (at ?drone ?from)))
  :effect (and (at end (not (at ?drone ?from)))
               (at end (at ?drone ?to))
               (at end (not (occupied ?from)))
               (at end (empty ?from))
               (at start (not (empty ?to)))
               (at start (occupied ?to))))

;only drone can do pick-up, from a known waypoint
(:durative-action pick-up
  :parameters (?drone - drone ?box - box ?air - airwaypoint ?ground - waypoint)
  :duration (= ?duration 1)
  :condition (and (at start (over ?air ?ground))
                  (over all (empty ?ground))
                  (over all (at ?drone ?air))
                  (over all (at ?box ?ground))
                  (over all (empty ?drone))
                  (over all (free ?box)))
  :effect (and (at start (not (at ?box ?ground)))
               (at end (carrying ?drone ?box))
               (at end (not (empty ?drone)))
               (at end (not (free ?box)))))

;(:durative-action pick-up
  ;:parameters (?turtlebot - turtlebot ?box - box ?waypoint - waypoint)
  ;:duration (= ?duration 10000)
  ;:condition (and (over all (at ?turtlebot ?waypoint))
                  ;(over all (at ?box ?waypoint))
                  ;(over all (empty ?turtlebot))
                  ;(over all (free ?box)))
  ;:effect (and (at start (not (at ?box ?waypoint)))
               ;(at end (carrying ?turtlebot ?box))
               ;(at end (not (empty ?turtlebot)))
               ;(at end (not (free ?box)))))

(:durative-action hand-over
  :parameters (?drone - drone ?turtlebot - turtlebot ?box - box ?air - airwaypoint ?ground - waypoint)
  :duration (= ?duration 1)
  :condition (and (at start (over ?air ?ground))
                  (over all (at ?drone ?air))
                  (over all (at ?turtlebot ?ground))
                  (at start (carrying ?drone ?box))
                  (at start (empty ?turtlebot)))
  :effect (and (at end (not (carrying ?drone ?box)))
               (at end (empty ?drone))
               (at end (carrying ?turtlebot ?box))
               (at start (not (empty ?turtlebot)))))

(:durative-action deliver
  :parameters (?drone - drone ?box - box ?air - airwaypoint ?ground - waypoint ?person - person)
  :duration (= ?duration 1)
  :condition (and (at start (over ?air ?ground))
                  (over all (empty ?ground))
                  (at start (carrying ?drone ?box))
                  (over all (at ?drone ?air))
                  (at start (at ?person ?ground)))
  :effect (and (at start (not (carrying ?drone ?box)))
               (at end (empty ?drone))
               (at end (at ?box ?ground))
               (at start (not (free ?box)))
               (at end (handled ?person))))

(:durative-action deliver
  :parameters (?turtlebot - turtlebot ?box - box ?waypoint - waypoint ?person - person)
  :duration (= ?duration 1)
  :condition (and (at start (carrying ?turtlebot ?box))
                  (over all (at ?turtlebot ?waypoint))
                  (at start (at ?person ?waypoint)))
  :effect (and (at start (not (carrying ?turtlebot ?box)))
               (at end (empty ?turtlebot))
               (at end (at ?box ?waypoint))
               (at start (not (free ?box)))
               (at end (handled ?person))))
)
