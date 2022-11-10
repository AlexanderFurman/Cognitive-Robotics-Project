;; Vehicle Routing Problem in PDDL
;; Borrowed from: 

(define (domain vrp)
  (:requirements :strips :typing)

  (:types freight location vehicle - object)

  (:predicates (at ?f - freight ?l - location)(at2 ?v - vehicle ?l - location)(adjacent ?l1 ?l2 - location)
               (in ?f - freight ?v - vehicle)(using ?v - vehicle)(destination ?f - freight ?l - location))


  (:action load
    :parameters (?f - freight ?v - vehicle ?l - location)
    :precondition (and (using ?v)
                       (at ?f ?l)
                       (at2 ?v ?l)
                       (>= (maxLoad) (+(load ?v) (weight ?f)))
                  )
    :effect (and (in ?f ?v)(not (at ?f ?l))(increase (cargo ?v) (weight ?f)))
  )
 )