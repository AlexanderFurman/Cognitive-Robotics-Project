(define (domain mars-domain)
 (:requirements :strips :typing :action-costs)
 (:types
    rover_0 location - object
    goal1 goal2 goal3 start_ - location
 )
 (:predicates (rover_at ?rover - rover_0 ?location - location) (visited ?rover - rover_0 ?location - location))
 (:functions (total-cost))
 (:action move_g1_g2
  :parameters ( ?r - rover_0 ?l_from - goal1 ?l_to - goal2)
  :precondition (and (rover_at ?r ?l_from))
  :effect (and (not (rover_at ?r ?l_from)) (rover_at ?r ?l_to) (visited ?r ?l_to) (increase (total-cost) 58)))
 (:action move_g1_g3
  :parameters ( ?r - rover_0 ?l_from - goal1 ?l_to_0 - goal3)
  :precondition (and (rover_at ?r ?l_from))
  :effect (and (not (rover_at ?r ?l_from)) (rover_at ?r ?l_to_0) (visited ?r ?l_to_0) (increase (total-cost) 26)))
 (:action move_g1_s
  :parameters ( ?r - rover_0 ?l_from - goal1 ?l_to_1 - start_)
  :precondition (and (rover_at ?r ?l_from))
  :effect (and (not (rover_at ?r ?l_from)) (rover_at ?r ?l_to_1) (visited ?r ?l_to_1) (increase (total-cost) 40)))
 (:action move_g2_g1
  :parameters ( ?r - rover_0 ?l_from_0 - goal2 ?l_to_2 - goal1)
  :precondition (and (rover_at ?r ?l_from_0))
  :effect (and (not (rover_at ?r ?l_from_0)) (rover_at ?r ?l_to_2) (visited ?r ?l_to_2) (increase (total-cost) 58)))
 (:action move_g2_g3
  :parameters ( ?r - rover_0 ?l_from_0 - goal2 ?l_to_0 - goal3)
  :precondition (and (rover_at ?r ?l_from_0))
  :effect (and (not (rover_at ?r ?l_from_0)) (rover_at ?r ?l_to_0) (visited ?r ?l_to_0) (increase (total-cost) 30)))
 (:action move_g2_s
  :parameters ( ?r - rover_0 ?l_from_0 - goal2 ?l_to_1 - start_)
  :precondition (and (rover_at ?r ?l_from_0))
  :effect (and (not (rover_at ?r ?l_from_0)) (rover_at ?r ?l_to_1) (visited ?r ?l_to_1) (increase (total-cost) 99)))
 (:action move_g3_g1
  :parameters ( ?r - rover_0 ?l_from_1 - goal3 ?l_to_2 - goal1)
  :precondition (and (rover_at ?r ?l_from_1))
  :effect (and (not (rover_at ?r ?l_from_1)) (rover_at ?r ?l_to_2) (visited ?r ?l_to_2) (increase (total-cost) 26)))
 (:action move_g3_g2
  :parameters ( ?r - rover_0 ?l_from_1 - goal3 ?l_to - goal2)
  :precondition (and (rover_at ?r ?l_from_1))
  :effect (and (not (rover_at ?r ?l_from_1)) (rover_at ?r ?l_to) (visited ?r ?l_to) (increase (total-cost) 30)))
 (:action move_g3_s
  :parameters ( ?r - rover_0 ?l_from_1 - goal3 ?l_to_1 - start_)
  :precondition (and (rover_at ?r ?l_from_1))
  :effect (and (not (rover_at ?r ?l_from_1)) (rover_at ?r ?l_to_1) (visited ?r ?l_to_1) (increase (total-cost) 67)))
 (:action move_s_g1
  :parameters ( ?r - rover_0 ?l_from_2 - start_ ?l_to_2 - goal1)
  :precondition (and (rover_at ?r ?l_from_2))
  :effect (and (not (rover_at ?r ?l_from_2)) (rover_at ?r ?l_to_2) (visited ?r ?l_to_2) (increase (total-cost) 40)))
 (:action move_s_g2
  :parameters ( ?r - rover_0 ?l_from_2 - start_ ?l_to - goal2)
  :precondition (and (rover_at ?r ?l_from_2))
  :effect (and (not (rover_at ?r ?l_from_2)) (rover_at ?r ?l_to) (visited ?r ?l_to) (increase (total-cost) 99)))
 (:action move_s_g3
  :parameters ( ?r - rover_0 ?l_from_2 - start_ ?l_to_0 - goal3)
  :precondition (and (rover_at ?r ?l_from_2))
  :effect (and (not (rover_at ?r ?l_from_2)) (rover_at ?r ?l_to_0) (visited ?r ?l_to_0) (increase (total-cost) 67)))
)
