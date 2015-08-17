;	*****
;	redundant-robot.lsp
;	*****

;	An 8-DOF redundant robot

;	Xavier A. Waller
;	EECE/ME 271
;	November 10, 2010
	

(defun make-redundant-robot nil 
;***************************************************
;;*************************************************
(make-serial-agent
;********
;Sphere1
(set-object 
	(make-revolute-link nil nil
		(make-sphere 8 20) (translate 0 0 0)
	) 
:color '( 0 0 153));Link1 wrt base
(rotatey 90)


;*************
;Truncated cone 1
(set-object 
	(make-revolute-link nil nil
		(make-truncated-cone 8 4 32 20) (translate 0 0  0)
	)
:color '(0 0 153));link2 wrt 1
(rotatey -90); rotates frame

;------------------------------------------------------------------------------------------------------------------
;********
;S2
(set-object 
	(make-revolute-link nil nil
		(make-sphere 4 20) (translate 0 0 0 )
	)
:color '( 255 255 0 ));Link3 wrt 2
(matmult (rotatey 90) (translate (- 32) 0 0))


;*************
;TC2
(set-object 
	(make-revolute-link nil nil
		(make-truncated-cone 4 2 16 20) (translate 0 0 0)
)
:color '(255 255 0));Link 4 wrt 3
(rotatey -90); rotates frame
;------------------------------------------------------------------------------------------------------------------
;********
;S3
(set-object 
	(make-revolute-link nil nil
		(make-sphere 2 20) (translate 0 0 0)
)
:color '( 255 153 0));Link5 wrt 4
(matmult (rotatey 90) (translate  (- 16) 0 0))  


;*************
;TC3
(set-object 
	(make-revolute-link nil nil
		(make-truncated-cone 2 1 8 20) (translate 0 0 0)
	)
:color '(255 153 0));Link 6 wrt 5
(rotatey -90); rotates frame

;------------------------------------------------------------------------------------------------------------------
;********
;S4
(set-object 
	(make-revolute-link nil nil
		(make-sphere 1  20) (translate 0 0 0 )
	)
:color '( 255 0 0));Link7 wrt 6
(matmult (rotatey 90) (translate (- 8) 0 0))


;*************
;TC4
(set-object 
	(make-revolute-link nil nil
		(make-truncated-cone 1 .5 4 20) (translate 0 0 0)
	)
:color '(255 0 0))
(rotatey -90); rotates frame
;link 8 wrt 7

);END MAKE-SERIAL
);END DEFUN
;***********
(clear-simulation)
(setq r (make-redundant-robot))
(use-objects r)
;******
