(define (problem SR1)
  (:domain social_robot)
  (:objects corr - location room - location)
  (:init
    (at corr)
    (face corr)
    (unknown (open corr room))
    ;(prefer_ask)
  )
  (:goal
    (at room)
  )
)

