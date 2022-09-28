;Header and description

(define (domain rover_environment)

    ;remove requirements that are not needed
    (:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)

    (:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle

        location locatable - object
        charge_station base cave - location
        rover cargo - locatable
        person supply - cargo

    )

    ; un-comment following line if constants are needed
    ;(:constants )

    (:predicates ;todo: define predicates here

        (rover ?r - rover)
        (location ?l - location)
        (at ?obj - locatable ?loc - location)
        (inside-rover ?c - cargo ?r - rover)
        (outside-rover ?c -cargo)
        (empty ?r - rover)
        (can-move ?r - rover ?l1 ?l2 - location)
        (rescued ?c)
        ; although any cargo listed as rescued, we can specify ppl to be rescued in problem file

    )


    (:functions ;todo: define numeric functions here

        ; (battery-capacity ?r - rover)
        ; (battery-consumption-rate)
        (battery-level ?r - rover)
        (mission-duration)
        (time-to-traverse ?l1 ?l2 - location)
        (people-rescued)

    )

    (:durative-action move
    ; TODO: change all variable changes to be parametric - not just "change by 10", etc.
    ; TODO: change variable changes based off of if cargo is present in the rover
        :parameters 
        (
            ?r - rover
            ?fromloc - location
            ?toloc - location
        )
        ; duration given in minutes
        :duration 
        (= ?duration (time-to-traverse ?fromloc ?toloc))

        :condition
            (and 
                (at start (rover ?r))
                (at start (location ?fromloc))
                (at start (location ?toloc))
                (over all (can-move ?r ?fromloc ?toloc))
                (at start (at ?r ?fromloc))
                (at end (> (battery-level ?r) 10))
                (at end (< (mission-duration) 240))
            )

        :effect
            (and
                (at start (decrease (battery-level ?r) 10))
                (at start (not (at ?r ?fromloc)))
                (at end (at ?r ?toloc))
                (at end (increase (mission-duration) ?duration))

                (forall (?c - cargo)
                    (when (and 
                        (at start (at ?c ?fromloc))
                        (at start (inside-rover ?c ?r))
                        )
                        (and
                        (at start (not (at ?c ?fromloc)))
                        (at end (at ?c ?toloc))
                        )
                    )
                )
            )


    )

    (:durative-action load-rover
        :parameters 
        (
            ?r - rover
            ?c - cargo
            ?l - location
        )

        :duration (= ?duration 1)

        :condition (and 
            (at start (empty ?r))
            (at start (outside-rover ?c))
            (at start (at ?c ?l))
            (at start (at ?r ?l))
            (at end (< (mission-duration) 240))
        )

        :effect (and
            (at end (not (empty ?r)))
            (at end (not (outside-rover ?c)))
            (at end (inside-rover ?c ?r))
            (at end (increase(mission-duration) ?duration))
        )
    )

    (:durative-action unload-rover
        :parameters (
            ?r - rover
            ?c - cargo
            ?l - location
        )

        :duration (= ?duration 1)

        :condition (and 
            (at start (not (empty ?r)))
            (at start (inside-rover ?c ?r))
            (at start (at ?c ?l))
            (at start (at ?r ?l))
            (at end (< (mission-duration) 240))
        )

        :effect (and 
            (at end (empty ?r))
            (at end (not (inside-rover ?c ?r)))
            (at end (outside-rover ?c))
            (at end (increase (mission-duration) ?duration))
        )
    )
    


)