;; Travelling Salesman Problem in PDDL
;; Borrowed from: https://github.com/SoarGroup/Domains-Planning-Domain-Definition-Language/blob/master/pddl/tsp-10.pddl

;; A TSP instance with ten cities.

(define (problem ten-cities)
  (:domain tsp)
  (:objects c1 c2 c3 c4 c5 c6 c7 c8 c9 c10)
  (:init (connected c1 c2) (connected c1 c3) (connected c2 c4)
         (connected c2 c5) (connected c3 c6) (connected c3 c7)
         (connected c4 c8) (connected c4 c9) (connected c5 c10)
         (connected c5 c1) (connected c6 c2) (connected c6 c3)
         (connected c7 c4) (connected c7 c5) (connected c8 c6)
         (connected c8 c7) (connected c9 c8) (connected c10 c1)
         (visited c1) (not-visited c2) (not-visited c3)
         (not-visited c4) (not-visited c5) (not-visited c6)
         (not-visited c7) (not-visited c8) (not-visited c9)
         (not-visited c10)
         (in c1) (starting c1) (not-complete))
  (:goal (and (visited c1) (visited c2) (visited c3) (visited c4)
              (visited c5) (visited c6) (visited c7) (visited c8)
              (visited c9) (visited c10) (complete)))
  )