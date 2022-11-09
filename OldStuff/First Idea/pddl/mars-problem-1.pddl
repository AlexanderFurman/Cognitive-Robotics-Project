(define (problem mars-problem-1) 

    (:domain rover-environment) 
    (:objects 
        waypoint1 waypoint2 waypoint3 waypoint4 - location
        rover1 - rover
        person1 - person
    )

    (:init 
        (= (mission-duration) 0)

        (= (battery-level rover1) 100) 

        (rover rover1) 

        (cargo person1) 

        (location waypoint1) (location waypoint2) (location waypoint3) (location waypoint4) 

        (path waypoint1 waypoint2) (= (time-to-traverse waypoint1 waypoint2) 10)
        ; (path waypoint1 waypoint3) (= (time-to-traverse waypoint1 waypoint3) 10)
        ; (path waypoint1 waypoint4) (= (time-to-traverse waypoint1 waypoint4) 14.142135623730951)
        (path waypoint2 waypoint1) (= (time-to-traverse waypoint2 waypoint1) 1)
        ; (path waypoint2 waypoint3) (= (time-to-traverse waypoint2 waypoint3) 14.142135623730951)
        ; (path waypoint2 waypoint4) (= (time-to-traverse waypoint2 waypoint4) 10)
        ; (path waypoint3 waypoint1) (= (time-to-traverse waypoint3 waypoint1) 10)
        ; (path waypoint3 waypoint2) (= (time-to-traverse waypoint3 waypoint2) 14.142135623730951)
        ; (path waypoint3 waypoint4) (= (time-to-traverse waypoint3 waypoint4) 10)
        ; (path waypoint4 waypoint1) (= (time-to-traverse waypoint4 waypoint1) 14.142135623730951)
        ; (path waypoint4 waypoint2) (= (time-to-traverse waypoint4 waypoint2) 10)
        ; (path waypoint4 waypoint3) (= (time-to-traverse waypoint4 waypoint3) 10)

        (at rover1 waypoint2) (at person1 waypoint1) 

        (empty rover1) 

        (outside-rover person1) 

    )

    (:goal 
        (at rover1 waypoint1) 

    ; (> (battery-level rover1) 0)

    ; (< (mission-duration) 240)
    )
)