(define (problem mars-problem)
 (:domain mars-domain)
 (:objects
   rover - rover_0
   g1 - goal1
   g2 - goal2
   g3 - goal3
   s - start_
 )
 (:init (rover_at rover s) (visited rover s) (= (total-cost) 0))
 (:goal (and (visited rover g1) (visited rover g2) (visited rover g3) (rover_at rover s)))
 (:metric minimize (total-cost))
)
