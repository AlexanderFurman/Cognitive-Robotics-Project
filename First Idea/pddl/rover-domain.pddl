;Header and description

(define (domain rover-environment)

    ;remove requirements that are not needed
    (:requirements :strips :fluents :typing :conditional-effects :negative-preconditions :equality)

    (:types 
        location locatable - object
        ; charge_station base cave - location
        rover cargo - locatable
        person supply - cargo
    )

    (:predicates
        (rover ?r - rover)
        (cargo ?c - cargo)
        (location ?l - location)
        (at ?obj - locatable ?loc - location)
        (inside-rover ?c - cargo ?r - rover)
        (outside-rover ?c -cargo)
        (empty ?r - rover)
        (path ?l1 ?l2 - location)
        (is-cave ?l - location)
        (is-charge-station ?l - location)
        (is-base ?l - location)

    )


    (:functions 
        (battery-capacity ?r - rover)
        (battery-consumption-rate)
        (battery-level ?r - rover)
        (mission-duration)
        (time-to-traverse ?l1 - location ?l2 - location)

    )

    (:action move
    ; TODO: change all variable changes to be parametric - not just "change by 10", etc.
    ; TODO: change variable changes based off of if cargo is present in the rover
        :parameters 
        (
            ?r - rover
            ?fromloc - location
            ?toloc - location
        )
        ; duration given in minutes
        ; :duration 
        ; (= ?duration (time-to-traverse ?fromloc ?toloc))

        :precondition
            (and 
                (rover ?r)
                (location ?fromloc)
                (location ?toloc)
                (path ?fromloc ?toloc)
                (at ?r ?fromloc)
                (> (battery-level ?r) 10)
                (< (mission-duration) 240)
            )

        :effect
            (and 
                (decrease (battery-level ?r) 10)
                (not (at ?r ?fromloc))
                (at ?r ?toloc)
                (increase (mission-duration) (time-to-traverse ?fromloc ?toloc))

                (forall (?c - cargo)
                    (when (and 
                        (cargo ?c)
                        (at ?c ?fromloc)
                        (inside-rover ?c ?r)
                        )
                        (and
                        (not (at ?c ?fromloc))
                        (at ?c ?toloc)
                        )
                    )
                )
            )
    )

    (:action load-rover
        :parameters 
        (
            ?r - rover
            ?c - cargo
            ?l - location
        )

        :precondition (and 
            (cargo ?c)
            (rover ?r)
            (location ?l)
            (empty ?r)
            (outside-rover ?c)
            (at ?c ?l)
            (at ?r ?l)
            (< (mission-duration) 240)
        )

        :effect (and 
            (not (empty ?r))
            (not (outside-rover ?c))
            (inside-rover ?c ?r)
            (increase(mission-duration) 1)
        )
    )

    (:action unload-rover
        :parameters (
            ?r - rover
            ?c - cargo
            ?l - location
        )

        :precondition (and 
            (cargo ?c)
            (rover ?r)
            (location ?l)
            (not (empty ?r))
            (inside-rover ?c ?r)
            (at ?c ?l)
            (at ?r ?l)
            (< (mission-duration) 240)
        )

        :effect (and 
            (empty ?r)
            (not (inside-rover ?c ?r))
            (outside-rover ?c)
            (increase (mission-duration) 1)
        )
    )
)