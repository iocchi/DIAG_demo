(define (domain social_robot)

  (:types location)

  (:predicates 
    (at ?x - location) 
    (face ?x - location) 
    (open ?x - location ?y - location)
    (prefer_ask)
  )


  (:action forward
    :parameters (?x - location ?y - location)
    :precondition (and (at ?x) (face ?y) (open ?x ?y))
    :effect (at ?y)
  )

  (:action backward
    :parameters (?x - location ?y - location)
    :precondition (and (at ?x) (not (face ?y)) (open ?x ?y))
    :effect (at ?y)
  )

  (:action turn
    :parameters (?x - location ?y - location)
    :precondition (and (at ?x) (not (face ?y)))
    :effect (face ?y)
  )

  (:action sense_open
    :parameters (?x - location ?y - location)
    :precondition (and (at ?x) (face ?y) (not (prefer_ask)))
    :observe (open ?x ?y)
  )

  (:action ask_open
    :parameters (?x - location ?y - location)
    :precondition (and (at ?x) (not (face ?y)) (prefer_ask))
    :observe (open ?x ?y)
  )

;  (:action cannot_do
;    :parameters (?x - location ?y - location)
;    :precondition (and (at ?x) (face ?y) (not (open ?x ?y)))
;    :effect (at ?y)
;  )

)

