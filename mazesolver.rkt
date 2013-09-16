;Robot maze travesal solver in Racket Christopher Brawn
;Robot maze traversal based on a binary heap priority queue.
;map values are encoded into a tuple '(x y w) with w=weight cost as defined by A* algorithm
;robot moves are based upon the smallest weight cost nodes in the priority queue
;nodes are added by the neighbour function which uses a set of rules to decide which squares/nodes
;to put in priority queue. A vector is used to hold the values of the nodes/squares already searched.
;search completes when goal is found our priority queue is empty, meaning there is no solution and thus returning #f
; a path list is returned.

;To run
;test for success
;(game (goal x y) (start x y) (size x y) '(list of x y lists for obstacles))
;(game '(4 4) '(1 1) '(5 5) '((2 2) (4 3) (3 1)))
;test for failure/unsolvable
;(game '(4 4) '(1 1) '(5 5) '((1 2) (2 2) (3 2) (4 2) (5 2)))

#lang racket
(require data/heap)


;to get elements out of width*height vectors
(define (get-Vector map x y size)
  (vector-ref map (+ (* (- y 1) (first size)) (- x 1))))

;to set a vector value
(define (set-Vector map x y size)
  (vector-set! map (+ (* (- y 1) (first size)) (- x 1)) 1))


;find distance to the goal using pythagoras method
(define (h-distance goalx goaly robotx roboty)
  (abs (+ (expt (- robotx goalx) 2) (expt(-  roboty goaly) 2))))


;get F()+H() and return updated node cost, this doesn't work correctly.
(define (get-node-weights node path goal)
  ;(print (length path))
  ;(newline)
         (list (first node)
                    (second node)
                    (+ (third node) (h-distance (first goal) (second goal) (first node) (second node)) (length path))
                       )
                    )


;check if nodes already in queue before inserting them, also update the visited vector
(define (pq-add pq inQueue mapping x y size goal path)
   (define node-weight (get-node-weights (get-Vector mapping x y size) path goal))
 ; (print node-weight)
  (if (eq? (get-Vector inQueue x y size) 0)
      (begin      
        (heap-add! pq node-weight)
        (set-Vector inQueue x y size)
        )
      #f))

;gets the orignal weighting of the node and adds it to the path
(define (add-to-path node map path size)
  (cons (list (first node) (second node) (third (get-Vector map (first node) (second node) size)))
        path))

;;solve the problem
; check if we are at goal, if so return path
;else add new neighbour squares and get new path node from heap
(define (solve-map start map path inQueue size goal)
  (cond ((equal? start goal) path)
        (else
   (add-neighbours map (first start) (second start) size inQueue goal path)
   ;check if heap is empty, if empty there is no solution return #f
   (cond
     ((eq? (heap-count pq) 0) #f)
     (else
   (define path-next (add-to-path (heap-min pq) map path size))
   (heap-remove-min! pq)
   (solve-map (list (first (car path-next)) (second (car path-next))) map path-next inQueue size goal))))))

;add neighbours to the priority queue
;rules outline ways to add neighbours
(define (add-neighbours map currentx currenty size inQueue goal path)
  (cond 
    ((and (> currentx 1) (not (eq? currentx (first size))) (> currenty 1) (not (eq? currenty (second size))))
        (begin
         (pq-add pq inQueue map currentx (- currenty 1) size goal path)   
                   (pq-add pq inQueue map currentx (+ currenty 1) size goal path)                 
                   (pq-add pq inQueue map (- currentx 1) currenty size goal path)
                   (pq-add pq inQueue map (+ currentx 1) currenty size goal path))
        )
    
        ((and (eq? currentx 1) (> currenty 1) (not (eq? currenty (second size))))
         (begin
         (pq-add pq inQueue map currentx (- currenty 1) size goal path)         
                   (pq-add pq inQueue map currentx (+ currenty 1) size goal path)                                     
                   (pq-add pq inQueue map (+ currentx 1) currenty size goal path))                            
                   )
        
        ((and (eq? currentx (first size)) (> currenty 1) (not (eq? currenty (second size))))
         (begin
         (pq-add pq inQueue map currentx (- currenty 1) size goal path)
                   (pq-add pq inQueue map currentx (+ currenty 1) size goal path)
                   (pq-add pq inQueue map (- currentx 1) currenty size goal path)
                   ))
        
        ((and (eq? currenty 1) (eq? currentx 1))
         (begin
         (pq-add pq inQueue map currentx (+ currenty 1) size goal path)
                    (pq-add pq inQueue map (+ currentx 1) currenty size goal path))                                     
                   )
        
         ((and (eq? currenty 1) (eq? currentx (first size))
          (begin
         (pq-add pq inQueue map currentx (+ currenty 1) size goal path)
                   (pq-add pq inQueue map (- currentx 1) currenty size goal path))            
                   ))
         
         ((and (eq? currenty (second size)) (eq? currentx 1))
          (begin
         (pq-add pq inQueue map currentx (- currenty 1) size goal path)
                   (pq-add pq inQueue map (+ currentx 1) currenty size goal path))               
         )
         
         ((and (eq? currenty (second size)) (eq? currentx (first size)))
          (begin
         (pq-add pq inQueue map currentx (- currenty 1) size goal path)
                   (pq-add pq inQueue map (- currentx 1) currenty size goal path))
         )
         
         ((eq? currenty (second size))
          (begin
          (pq-add pq inQueue map currentx (- currenty 1) size goal path)
                     (pq-add pq inQueue map (- currentx 1) currenty size goal path)
                     (pq-add pq inQueue map (+ currentx 1) currenty size goal path)))               
          
         
         ((eq? currenty 1)
          (begin
          (pq-add pq inQueue map currentx (+ currenty 1) size goal path)
                     (pq-add pq inQueue map (- currentx 1) currenty size goal path)
                     (pq-add pq inQueue map (+ currentx 1) currenty size goal path)))))                 
         
;print out heap, blows away heap, just for testing
(define (print-heap heap)
  (cond ((eq? (heap-count heap) 0) '())
        (else 
         (print (heap-min heap))
         (newline)
         (heap-remove-min! heap)
         (print-heap heap))))



;remove smallest from heap, does not return yet.
(define (pick-smallest queue path)
  (begin
  (print (append (heap-min queue) path))
   (heap-remove-min! queue)
   )
  )
  
;make path vector with update cost values to keep track of cost to home
(define (cost-to-home path)
  (cond ((null? path) 0)
        ((+ (third (car path)) (cost-to-home(cdr path))))))

;custom comparator for the heap data structure
(define (compare-h v1 v2)
  (cond
    ((eqv? (third v1) (third v2)) 0)
    ((< (third v1) (third v2)) #t)  
    ((> (third v1) (third v2)) #f)))

;priority queue 
(define pq (make-heap compare-h))


;generate map vector based on user input size
(define (make-map size)
  (begin
  (define map-vector (make-vector (* (first size) (second size))))
  (fill map-vector size 1 1)
  ))

;;create obstacle queue from input list and start position
(define (make-inQueue size obstacles start)
  (begin
  (define map-inQ (make-vector (* (first size) (second size))))
  (fill-inQueue map-inQ (cons start obstacles) size)))

;;create inital map based on user size input
(define (fill map size x y)
  (cond
    ((and (eq? (first size) x) (eq? (second size) y))
     (begin
       (vector-set! map (-(* (first size)(second size)) 1) (list x y 1))
       map))
    (else 
     (cond
       ((eq? (first size) x)
         (vector-set! map (+ (* (- y 1) (first size)) (- x 1)) (list x y 1))
        (fill map size 1 (+ y 1)))
        (else
     (begin
       (vector-set! map (+ (* (- y 1) (first size)) (- x 1)) (list x y 1))
              (fill map size (+ x 1) y)
              )
     )))))

;;create obstacle queue, also used to see if square has already been searched
(define (fill-inQueue q obstacles size)
  (cond 
    ((null? obstacles) q)
    (else
     (begin
       (define x (first (first obstacles)))
       (define y (second (first obstacles)))
     (vector-set! q (+ (* (- y 1) (first size)) (- x 1)) 1)
     (fill-inQueue q (cdr obstacles) size))    
    )) 
  )

;empty heap at end of program as heap is not deleted until environment is run again
(define (emptyheap)
  (cond
    ((eq? (heap-count pq) 0) #t)
    (else
     (begin
     (heap-remove-min! pq)
     (emptyheap)))))



;create and solve map. Need to check goal weights, if working properly.
;test for success
;(game (goal x y) (start x y) (size x y) '(list of x y lists for obstacles))
;(game '(4 4) '(1 1) '(5 5) '((2 2) (4 3) (3 1)))
;test for failure/unsolvable
;(game '(4 4) '(1 1) '(5 5) '((1 2) (2 2) (3 2) (4 2) (5 2)))
(define (game goal start size obstacles)
  (define game-map (make-map size))
  (define game-inQueue (make-inQueue size obstacles start))
  (define solution? (solve-map start game-map '() game-inQueue size goal))
  (emptyheap)
  ;check if a solution is found, if not #f, else return list
  (cond
  ((eq? solution? #f) #f)
  (else
  (define trimmed-list(map 
                       (lambda (xs) (list (first xs) (second xs)))
                       solution?))
  (cons start(reverse trimmed-list))
  )))

(define (clean-path path)
  (cond
    ((null? (cdr path)) path)
    ((> (abs(- (first (car path)) (first (car (cdr path))))) 1) (clean-path (cdr path)))
    ((> (abs(- (second (car path)) (second (car (cdr path))))) 1) (clean-path (cdr path)))
    (else
     (cons (car path) (clean-path (cdr path))))))







  


