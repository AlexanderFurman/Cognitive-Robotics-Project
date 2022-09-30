(define (problem mars-problem-0)

(:domain rover-environment)
(:objects waypoint1 waypoint2 waypoint3 waypoint4 waypoint5 waypoint6 waypoint7 waypoint8 waypoint9 rover1 person1 )

(:init 
(= (mission-duration) 0)

(= (battery-level rover1) 100) 

(rover rover1) 

(cargo person1) 

(location waypoint1) (location waypoint2) (location waypoint3) (location waypoint4) (location waypoint5) (location waypoint6) (location waypoint7) (location waypoint8) (location waypoint9) 

(path waypoint1 waypoint2) (= (time-to-traverse waypoint1 waypoint2) 10)
(path waypoint1 waypoint4) (= (time-to-traverse waypoint1 waypoint4) 10)
(path waypoint1 waypoint5) (= (time-to-traverse waypoint1 waypoint5) 14.142135623730951)
(path waypoint2 waypoint1) (= (time-to-traverse waypoint2 waypoint1) 10)
(path waypoint2 waypoint3) (= (time-to-traverse waypoint2 waypoint3) 10)
(path waypoint2 waypoint4) (= (time-to-traverse waypoint2 waypoint4) 14.142135623730951)
(path waypoint2 waypoint5) (= (time-to-traverse waypoint2 waypoint5) 10)
(path waypoint2 waypoint6) (= (time-to-traverse waypoint2 waypoint6) 14.142135623730951)
(path waypoint3 waypoint2) (= (time-to-traverse waypoint3 waypoint2) 10)
(path waypoint3 waypoint5) (= (time-to-traverse waypoint3 waypoint5) 14.142135623730951)
(path waypoint3 waypoint6) (= (time-to-traverse waypoint3 waypoint6) 10)
(path waypoint4 waypoint1) (= (time-to-traverse waypoint4 waypoint1) 10)
(path waypoint4 waypoint2) (= (time-to-traverse waypoint4 waypoint2) 14.142135623730951)
(path waypoint4 waypoint5) (= (time-to-traverse waypoint4 waypoint5) 10)
(path waypoint4 waypoint7) (= (time-to-traverse waypoint4 waypoint7) 10)
(path waypoint4 waypoint8) (= (time-to-traverse waypoint4 waypoint8) 14.142135623730951)
(path waypoint5 waypoint1) (= (time-to-traverse waypoint5 waypoint1) 14.142135623730951)
(path waypoint5 waypoint2) (= (time-to-traverse waypoint5 waypoint2) 10)
(path waypoint5 waypoint3) (= (time-to-traverse waypoint5 waypoint3) 14.142135623730951)
(path waypoint5 waypoint4) (= (time-to-traverse waypoint5 waypoint4) 10)
(path waypoint5 waypoint6) (= (time-to-traverse waypoint5 waypoint6) 10)
(path waypoint5 waypoint7) (= (time-to-traverse waypoint5 waypoint7) 14.142135623730951)
(path waypoint5 waypoint8) (= (time-to-traverse waypoint5 waypoint8) 10)
(path waypoint5 waypoint9) (= (time-to-traverse waypoint5 waypoint9) 14.142135623730951)
(path waypoint6 waypoint2) (= (time-to-traverse waypoint6 waypoint2) 14.142135623730951)
(path waypoint6 waypoint3) (= (time-to-traverse waypoint6 waypoint3) 10)
(path waypoint6 waypoint5) (= (time-to-traverse waypoint6 waypoint5) 10)
(path waypoint6 waypoint8) (= (time-to-traverse waypoint6 waypoint8) 14.142135623730951)
(path waypoint6 waypoint9) (= (time-to-traverse waypoint6 waypoint9) 10)
(path waypoint7 waypoint4) (= (time-to-traverse waypoint7 waypoint4) 10)
(path waypoint7 waypoint5) (= (time-to-traverse waypoint7 waypoint5) 14.142135623730951)
(path waypoint7 waypoint8) (= (time-to-traverse waypoint7 waypoint8) 10)
(path waypoint8 waypoint4) (= (time-to-traverse waypoint8 waypoint4) 14.142135623730951)
(path waypoint8 waypoint5) (= (time-to-traverse waypoint8 waypoint5) 10)
(path waypoint8 waypoint6) (= (time-to-traverse waypoint8 waypoint6) 14.142135623730951)
(path waypoint8 waypoint7) (= (time-to-traverse waypoint8 waypoint7) 10)
(path waypoint8 waypoint9) (= (time-to-traverse waypoint8 waypoint9) 10)
(path waypoint9 waypoint5) (= (time-to-traverse waypoint9 waypoint5) 14.142135623730951)
(path waypoint9 waypoint6) (= (time-to-traverse waypoint9 waypoint6) 10)
(path waypoint9 waypoint8) (= (time-to-traverse waypoint9 waypoint8) 10)

(at rover1 waypoint5) (at person1 waypoint1) 

(empty rover1) 

(outside-rover person1) 

)

(:goal 
(at person1 waypoint5) 

(> (battery-level rover1) 0) 

(< (mission-duration) 240)
)
)