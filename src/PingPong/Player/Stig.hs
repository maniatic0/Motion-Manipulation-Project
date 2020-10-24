{-# LANGUAGE ScopedTypeVariables #-}

-- By Christian Oliveros and Minmin Chen
module PingPong.Player.Stig -- (stig) 
where

import Control.Lens (view, (&), (.~), (^.))
import Data.Bool (bool)
import Data.Ext
import Data.Geometry hiding (head)
import Data.Maybe
import Debug.Trace
import GHC.Float
import qualified Numeric.LinearAlgebra as Numerical
import PingPong.Model
import PingPong.Player.Stig.General
import PingPong.Player.Stig.GeometryHelpers
import PingPong.Player.Stig.Kinematics
import PingPong.Player.Stig.Fabrik

import Control.Monad

-- | Simulation's gravity
simulationGravity :: Float
simulationGravity = 2

-- | Simulation's Table Height
simulationTableHeight :: Float
simulationTableHeight = 0.5

-- | Simulation's Table Center X Position
simulationTableCenterX :: Float
simulationTableCenterX = 0

-- | Simulation's Table Max X Position
simulationTableMaxX :: Float
simulationTableMaxX = 1

-- | Normalized direction of the table
simulationTableDir :: Vector 2 Float
simulationTableDir = Vector2 (-1) 0

-- | Bat's Length
simulationBatLength :: Float
simulationBatLength = 0.1

-- | Predict the time (relative to current time) the gravity parabole is going to intersect a height
predictFreefallHeightInter :: Point 2 Float -> Vector 2 Float -> Float -> Maybe Float 
predictFreefallHeightInter p v tH = bool res Nothing (isNothing possible || null tsRaw) 
  where
    a2 = -simulationGravity / 2
    a1 = view yComponent v
    a0 = view yCoord p - tH
    possible = solveQuadratic a2 a1 a0
    tsRaw = fromJust possible
    -- If solveQuadratic is [] change for 0, because all t are valid
    -- Also, make sure the times are valid between 0 and 1 (threshold is used for approximations)
    ts = filter (0 <=) $ map (globalThreshold 0) $ bool tsRaw [0] (null tsRaw)
    res = case possible of
        Nothing -> Nothing
        Just _ -> case ts of
                    [] -> Nothing -- No solution
                    _ -> Just $ minimum ts -- Minimum Valid Time
      

-- | Evaluate the free fall formulas for a time
freeFallEvaluateTime :: Point 2 Float -> Vector 2 Float -> Float -> (Point 2 Float, Vector 2 Float)
freeFallEvaluateTime p v t = (Point2 x y, Vector2 vx vy)
  where
    vx = view xComponent v
    x = vx * t + view xCoord p

    vy0 = view yComponent v
    vy = -simulationGravity * t + vy0
    y = -simulationGravity * t * t / 2 + vy0 * t + view yCoord p

-- | Reflect a velocity vector againts the table like in a collision
reflectVelocityTable :: Vector 2 Float -> Vector 2 Float
reflectVelocityTable v = reflect v simulationTableDir

-- | Get the normal of the free fall velocity
freefallNormal :: Vector 2 Float -> Vector 2 Float
freefallNormal v = n
  where
    nUnorm = Vector2 (-view yComponent v) (view xComponent v)
    (n, _) = normalizeVector nUnorm

-- | Get the (relative) time of maximum height of a freefall. Note, it might be negative, meaning it already peaked
freeFallMaxHeightTime :: Vector 2 Float -> Float
freeFallMaxHeightTime v = view yComponent v / simulationGravity

-- | Calculates the (relative) time to reach a x position in a freefall. Note, it might be negative, meaning it already passed
freeFallTimeToXPos :: Point 2 Float -> Vector 2 Float -> Float -> Float
freeFallTimeToXPos p v x = (x - view xCoord p) / view xComponent v

-- | Predict the info of the ball bouncing on our side of the table
predictTableBounce :: Point 2 Float -> Vector 2 Float -> Maybe (Point 2 Float, Vector 2 Float, Float)
predictTableBounce p v = res
  where 
    tPossible = predictFreefallHeightInter p v simulationTableHeight
    (pB, vB) = freeFallEvaluateTime p v (fromJust tPossible)
    xPos = view xCoord pB
    xPosValid =  simulationTableCenterX <= xPos && xPos <= simulationTableMaxX
    res = case tPossible of
            Nothing -> Nothing
            Just t -> if xPosValid then return (pB, reflectVelocityTable vB, t) else Nothing


-- | Checks if a position is valid for our side of the table
checkValidPos :: Point 2 Float -> Bool
checkValidPos p0 = simulationTableCenterX <= xPos && xPos <= simulationTableMaxX && simulationTableHeight <= yPos
  where
    xPos = view xCoord p0
    yPos = view yCoord p0

-- | Try to predict the best interception time
predictBestInterceptionTime :: Point 2 Float -> Vector 2 Float -> Maybe Float
predictBestInterceptionTime p v = res
  where
    ts = [freeFallMaxHeightTime v, freeFallTimeToXPos p v simulationTableMaxX, freeFallTimeToXPos p v (simulationTableMaxX / 2), freeFallTimeToXPos p v (simulationTableMaxX / 4)]
    tF = filter tFilter ts

    res =  if null tF then Nothing else Just $ minimum tF

    tFilter :: Float -> Bool
    tFilter t = 0 <= t && checkValidPos p0
      where
        (p0, _) = freeFallEvaluateTime p v t



-- | Place the bat normal to a ball point and velocity
placeBatInBounceCurve :: Point 2 Float -> Vector 2 Float -> LineSegment 2 () Float
placeBatInBounceCurve p v = line
  where
    
    -- Normal to Velocity
    --n = freefallNormal v
    (n, _) = normalizeVector v
    p0 = p .+^ (n ^* (simulationBatLength / 2))
    p1 = p .-^ (n ^* (simulationBatLength / 2))
    p0Dist = distToSpecialBase p0
    p1Dist = distToSpecialBase p1
    
    line = bool (ClosedLineSegment (p1 :+ ()) (p0 :+ ())) (ClosedLineSegment (p0 :+ ()) (p1 :+ ()))  (p0Dist < p1Dist)

    -- | Distance to Stig base at table height
    distToSpecialBase :: Point 2 Float -> Float
    distToSpecialBase po = norm $ po .-. Point2 stigFoot simulationTableHeight


-- | Try to intercept Ball
tryInterceptBall :: Arm -> Point 2 Float -> Vector 2 Float -> Float -> IO Motion
tryInterceptBall arm p v tColl =
   do 
      -- Other player did a proper hit we have to respond to 
      let tPossible = predictBestInterceptionTime p v

      let toRest = armToStigRestMotion arm

      case tPossible of
        Nothing -> return $ trace ("Opponent did a proper hit we can't intercept at " ++ show tColl) applyMotionLimits toRest
        Just tP -> do
          let (pM, vM) = freeFallEvaluateTime p v tP
          let bat = placeBatInBounceCurve pM vM

          -- this is just in the case we can reach it
          mIntercept <- stigPlanSeg stigFoot arm bat

          case mIntercept of
            [] -> return $ trace ("Opponent did a proper hit we can't catch at " ++ show tColl ++ " " ++ show bat) applyMotionLimits toRest -- Velocity limits
            _ -> do
              let mV = armToMotion arm mIntercept
              return $ trace ("Opponent did a proper hit we can catch at " ++ " " ++ show bat) applyMotionLimits mV -- Velocity limits

-- | Stig's player
stig :: Player
stig =
  Player
    { 
      name = "Stig",
      arm = stigArm,
      initArm = stigArm,
      foot = stigFoot,
      prepare = return (),
      terminate = return (),
      action = stigAction,
      collide = stigCollide,
      planPnt = stigPlanPnt,
      planSeg = stigPlanSeg,
      stretch = \_ _ -> return stigNoMotion,
      dance   = \_ _ -> return stigNoMotion
    }

-- | Arm to use
stigArm :: Arm
stigArm =
  checkArm
    [ Link paleBlue 0.5,
      Joint red (-0.3), -- (0.1)
      Link paleBlue 0.4,
      Joint red 1.3, -- (0.1)
      Link paleBlue 0.3,
      Joint red 0.9, -- (-0.1)
      Link paleBlue 0.2,
      Joint red 0.5, -- (-0.1)
      Link hotPink 0.1 -- Bat
    ]

-- | Stig Arm Length
stigArmLength :: Float
stigArmLength = armLength stigArm

-- | Separation from the center of the table
stigFoot :: Float
stigFoot = 1.5

-- | Stig rest postion
stigRest :: Motion
stigRest = getCurrentJoints stigArm

-- | Get the a zeroed Motion list for Stig's arm
stigNoMotion :: Motion
stigNoMotion = map f stigRest
  where
    f = const 0

-- | Calculate Motion Velocity to Rest Motion. !Warning: no limits are applied
armToStigRestMotion :: Arm -> Motion
armToStigRestMotion ar = armToMotion ar stigRest 

-- | Check collision of moving line and point
stigCollide ::
  forall r.
  (Num r, Floating r, Ord r, Eq r, Show r) =>
  (r, Point 2 r, LineSegment 2 () r) ->
  (r, Point 2 r, LineSegment 2 () r) ->
  IO (Point 2 r)
stigCollide t1 t2 = bool (error "Stig Collide Failed a Test Case") (return (f t1 t2)) completeCheck
  where
    f = movingBallMovingLineCollide
    generateTestState :: (Num r, Floating r, Ord r) => r -> (r, r) -> (r, r) -> (r, r) -> (r, Point 2 r, LineSegment 2 () r)
    generateTestState t (px, py) (pl0x, pl0y) (pl1x, pl1y) = (t, Point2 px py, ClosedLineSegment (Point2 pl0x pl0y :+ ()) (Point2 pl1x pl1y :+ ()))

    testCases =
      [ (Point2 0 0, generateTestState 0 (0, 0) (1, -1) (1, 1), generateTestState 1 (0, 0) (1, -1) (1, 1)),
        (Point2 0 2, generateTestState 0 (0, 0) (0, 0) (1, 0), generateTestState 1 (0, 0) (0, 1) (1, 1)),
        (Point2 0 0, generateTestState 0 (0, 0) (1, -1) (1, 1), generateTestState 1 (2, 0) (1, -1) (1, 1)),
        (Point2 (-1) 0, generateTestState 0 (0, 0) (1, -1) (1, 1), generateTestState 1 (1, 0) (0, -1) (0, 1)),
        (Point2 1 1, generateTestState 0 (0, 0) (0, -1) (2, 1), generateTestState 1 (2, 0) (0, -1) (2, 1)),
        (Point2 (-1) 1, generateTestState 0 (0, 0) (0, -1) (2, 1), generateTestState 1 (0, 0) (-2, -1) (0, 1)),
        (Point2 1.2 0.8, generateTestState 0 (0.3, 1.0) (0.1, 2.1) (-0.5, 0.9), generateTestState 1 (1.2, 0.8) (-0.2, 2.2) (-0.3, 1.1))
        --(Point2 (-6) (-4), generateTestState 0 (-5, -5) (0, 1) (1, 0), generateTestState 1 (5, 7) (0, 1) (1, 0))
      ]

    checkCollision :: (Num r, Floating r, Ord r, Show r) => (Point 2 r, (r, Point 2 r, LineSegment 2 () r), (r, Point 2 r, LineSegment 2 () r)) -> (Bool, Diff (Point 2) r, Point 2 r)
    checkCollision (ans, s1, s2) = (diffX == 0 && diffY == 0, diff, c)
      where
        c = f s1 s2
        diff = c .-. ans
        diffX = globalThreshold 0 $ abs $ view xComponent diff
        diffY = globalThreshold 0 $ abs $ view yComponent diff

    performTest :: (Num r, Floating r, Ord r, Show r) => (Point 2 r, (r, Point 2 r, LineSegment 2 () r), (r, Point 2 r, LineSegment 2 () r)) -> Bool
    performTest testCase@(ans, s1, s2) = bool (error showError) True correct
      where
        (correct, diff, p) = checkCollision testCase
        showError =
          "Expected " ++ show ans ++ " but got "
            ++ show p
            ++ " with Diff "
            ++ show diff
            ++ " \n\tFor case:\n\t"
            ++ show s1
            ++ "\n\t->\n\t"
            ++ show s2

    completeCheck = all performTest testCases

--test = stigCollide (0, Point2 0 0, ClosedLineSegment (Point2 0 0 :+ ()) (Point2 1 0 :+ ())) (1, Point2 0 0, ClosedLineSegment (Point2 1 0 :+ ()) (Point2 1 0 :+ ()))
--test = stigCollide (0, Point2 (-1) 1, ClosedLineSegment (Point2 0 0 :+ ()) (Point2 1 1 :+ ())) (1, Point2 0 0, ClosedLineSegment (Point2 0 0 :+ ()) (Point2 (-1) 1 :+ ()))
--test = stigCollide (0, Point2 (-1) 1, ClosedLineSegment (Point2 0 (-1) :+ ()) (Point2 1 1 :+ ())) (1, Point2 0 0, ClosedLineSegment (Point2 0 (-1) :+ ()) (Point2 (-1) 1 :+ ()))

stigAction :: Float -> (Float, Item) -> BallState -> Arm -> IO Motion
stigAction _ (tColl, Other _) _ arm =
  return $
    -- Ball hit something out of the game, this means someone scored
    -- Go to rest
    let toBase = armToMotion arm stigNoMotion
     in trace ("Someone Scored at " ++ show tColl) applyMotionLimits toBase -- Velocity limits
stigAction _ (tColl, Bat Self) _ arm =
  return $
    -- We hit the ball, go to rest motion
    let toRest = armToStigRestMotion arm
     in trace ("We just hit the ball at " ++ show tColl) applyMotionLimits toRest -- Velocity limits
stigAction _ (tColl, Table Opponent) _ arm =
  return $
    -- Our hit was correct and we reached the other player's side
    -- So rest
    let toRest = armToStigRestMotion arm
     in trace ("We did a proper hit at " ++ show tColl) applyMotionLimits toRest -- Velocity limits
stigAction t (tColl, Air) bs arm =
  return $
    -- IInvalid State
    let toBase = armToMotion arm stigNoMotion
     in trace ("Impossible State " ++ show tColl) applyMotionLimits toBase -- Velocity limits
stigAction t (tColl, Table Self) bs arm =
    do 
      -- Other player did a proper hit we have to respond to 
      -- Distance to max point for now
      let p = loc bs
      let v = dir bs
      tryInterceptBall arm p v tColl
stigAction t (tColl, Bat Opponent) bs arm =
  do 
      -- this is just in the case we can reach it
      let toRest = armToStigRestMotion arm

      -- Other player did a hit we have to respond to 
      let p = loc bs
      let v = dir bs

      let mayBounce = predictTableBounce p v

      case mayBounce of
        Nothing -> return $ trace ("Opponent did a wrong hit at " ++ show tColl) applyMotionLimits (armToMotion arm stigNoMotion) -- Velocity limits
        Just (pT, vT, _) -> tryInterceptBall arm pT vT tColl

-- | Stig Plan Threshold
stigPlanThreshold :: (Num r, Ord r, Fractional r) => r -> r -> r
stigPlanThreshold = threshold 0.1

-- | Calculates the possible motion values to achieve a point. If it fails it returns []
stigPlanPnt :: Float -> Arm -> Point 2 Float -> IO Motion
stigPlanPnt foot arm p
  | isTooFar = trace "Too Far" return []
  | stigPlanThreshold 0 eB == 0 = return $ map normalizeAngle qB -- \$ jointVectorToMotion qB
  | otherwise = trace ("Error: " ++ show eB) return []
  where
    isTooFar = norm (p .-. Point2 foot 0) > 1.2 * armLength arm
    {- (a, m) = getArmKinematicAndMotion foot arm
    q = motionToJointVector m
    (qB, eB) = newtonRaphsonIK a (homogeneousZero, pointToHomogenousPoint p) q -}
    (qB, eB) = fabrikToPoint foot arm p

-- | Calculates the possible motion values to achieve a line segment. If it fails it returns []
stigPlanSeg :: Float -> Arm -> LineSegment 2 () Float -> IO Motion
stigPlanSeg foot arm s
  | isOnlyBat = return [] -- No idea if we reach it because we can't move
  | isOnlyBatJoint = return $ bool [] onlyBatJointQ onlyBatJointCheck -- If we can rotate the only useful joint to match the end point
  | normalCheck = return $ map normalizeAngle qF
  | otherwise = trace ("Errors" ++ show (eB, eBat)) return []
  where
    -- Target Info
    startPoint = s ^. (start . core)
    p0 = pointToHomogenousPoint startPoint
    endPoint = s ^. (end . core)
    p1 = pointToHomogenousPoint endPoint

    -- Simplifyied arm
    simplerArm = simplifyArm arm

    -- Fix the bat and all the joints until a new
    revArm = reverse simplerArm -- Inverted arm with the bat at the start
    bat = head revArm -- The first one is the Bat for segment
    batLength = getLinkLength bat -- Bat Length

    -- Batless arm. The first element is a joint (or nothing)
    restArmRev = tail revArm

    -- Joints between original bat and the next link to use as a bat for the rest
    firstJointsRev = takeWhile isJoint restArmRev

    -- The arm is a bat (or can be simplified to that) and nothing else
    isOnlyBat = null firstJointsRev

    -- Small arm with a bat for first link. Note that it can be empty
    smallRev = dropWhile isJoint restArmRev

    -- Small from base to new bat
    smallArm = reverse smallRev

    -- The arm is a bat and a joint (or can be simplified to that) and nothing else
    isOnlyBatJoint = null smallArm

    -- Base to End point of Segment
    onlyBatJointBaseToP1 = p1 - homogeneousPoint (float2Double foot) 0
    (onlyBatJointBaseToP1Normalized, onlyBatJointBaseToP1Norm) = normalize2D onlyBatJointBaseToP1

    -- (onlyBatJointAngle, _) = newtonRaphsonAcos onlyBatJointAngleFirstApprox onlyBatJointAngleCos
    onlyBatJointAngle = angleVector homogeneousVectorY onlyBatJointBaseToP1Normalized
    onlyBatJointQ = reverse $ setFirstJointRest0 onlyBatJointAngle firstJointsRev

    -- Check that the bat has the same length as the distance from base to endpoint
    onlyBatJointCheck = stigPlanThreshold 0 (float2Double batLength - onlyBatJointBaseToP1Norm) == 0

    -- From here we are in a normal case we have at least this form: link (bat) -- joint -- link -- base
    -- Small Arm must reach the start point of segment
    (a, m) = getArmKinematicAndMotion foot smallArm
    q = motionToJointVector m
    (qB, eB) = newtonRaphsonIK a (homogeneousZero, p0) q
    mB = jointVectorToMotion qB

    -- Small Arm Forward Transforms
    fwdTIB = applyForwardKinematicTransInv a qB
    fwdTBMatInv = applyForwardKinematicMatrixTransInv fwdTIB

    -- p1Local Position
    p1Local = fwdTBMatInv Numerical.#> p1

    -- Vector from small bat to target
    smallBatToP1 = p1Local - homogeneousZero
    (smallBatToP1Normalized, smallBatToP1Norm) = normalize2D smallBatToP1

    smallBatAngle = angleVector homogeneousVectorX smallBatToP1Normalized

    qF = mB ++ reverse (setFirstJointRest0 smallBatAngle firstJointsRev)

    endP = rotateTrans smallBatAngle Numerical.#> homogeneousPoint (float2Double batLength) 0
    eBat = Numerical.norm_2 (p1Local - endP)

    -- Check that small bat is at p0 and that we can reach p1
    normalCheck =
      stigPlanThreshold 0 eB == 0
        && stigPlanThreshold 0 eBat == 0
        && (stigPlanThreshold 0 (float2Double batLength - smallBatToP1Norm) == 0)

    isJoint :: Element -> Bool
    isJoint (Joint _ _) = True
    isJoint _ = False
    getLinkLength :: Element -> Float
    getLinkLength (Link _ t) = t
    setFirstJointRest0 :: Double -> Arm -> Motion
    setFirstJointRest0 q as = double2Float q : map (const 0) (tail as)
    cross2D :: Numerical.Vector Numerical.R -> Numerical.Vector Numerical.R -> Numerical.R
    cross2D v1 v2 = Numerical.atIndex (Numerical.cross v1 v2) 2
    normalize2D :: Numerical.Vector Numerical.R -> (Numerical.Vector Numerical.R, Numerical.R)
    normalize2D v
      | globalThreshold 0 vNorm == 0 = (homogeneousZero, 0)
      | otherwise = (v / Numerical.scalar vNorm, vNorm)
      where
        vNorm = Numerical.norm_2 v

    angleVector :: Numerical.Vector Numerical.R -> Numerical.Vector Numerical.R -> Numerical.R
    angleVector v1 v2 = atan2 (cross2D v1 v2) (Numerical.dot v1 v2)


-- Testing Starts Here
-- | Create a Test Case for stigPlanPnt
createPlanPntCase :: Float -> Arm -> (Float, Float) -> Motion -> (Float, Arm, Point 2 Float, Motion)
createPlanPntCase f a (xT, yT) m = (f, a, Point2 xT yT, m)

-- | Test stigPlanPnt
testPlanPnt :: (Float, Arm, Point 2 Float, Motion) -> IO Bool
testPlanPnt (f, arm, pT, mT)
  = do
      -- Expected Position
      let qT = motionToJointVector mT
      let xTargetGlobal = pointToHomogenousPoint pT

      -- Arm
      let (a, _) = getArmKinematicAndMotion f arm

      -- Calculate Answer
      mB <- stigPlanPnt f arm pT
      if null mB then
        return $ bool (trace "Wrong Null" False) (null mT) (null mT)
      else
        do
          let qB = motionToJointVector mB

          -- Forward Transforms
          let fwdTT = applyForwardKinematicTrans a qT
          let fwdTB = applyForwardKinematicTrans a qB

          -- Bat Global Position
          let batGlobalB = applyForwardKinematicMatrixTrans fwdTB Numerical.#> homogeneousZero
          let batGlobalT = applyForwardKinematicMatrixTrans fwdTT Numerical.#> homogeneousZero

          -- Error
          let eB = xTargetGlobal - batGlobalB
          let eNormB = Numerical.norm_2 eB

          let eT = trace ("Best " ++ show (batGlobalB, eNormB, mB)) $ xTargetGlobal - batGlobalT
          let eNormT = Numerical.norm_2 eT

          let result = trace ("Target " ++ show (batGlobalT, eNormT, mT)) $ (globalThreshold 0 eNormB == 0) && (eNormB <= eNormT)
          return $ result && (length mB == length mT)
 
-- | Test Cases for testPlanPnt
planPntTestCases :: [(Float, Arm, Point 2 Float, Motion)]
planPntTestCases =
  [ createPlanPntCase
      0
      [
        Joint red 0.0,
        Link red 0.1
      ]
      (0, 0.1)
      [0],
    createPlanPntCase
      0
      [
        Joint red 0.0,
        Link red 0.1
      ]
      (0.1, 0)
      [pi/2],
    createPlanPntCase
      1.5
      [ Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.1
      ]
      (1.22385, 0.80917)
      [0.1, 0.2, 0.3, 0.4],
    createPlanPntCase
      1.5
      [ Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.1
      ]
      (1.77615, 0.80917)
      [-0.5, 0.0, 0.4, 0.6],
    createPlanPntCase
      1.5
      [ Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.1
      ]
      (1.48013, 0.89202)
      [0.1, -0.2, 0.3, -0.4],
      createPlanPntCase
      1.5
      [ Link red 0.1,
        Joint red 0.0,
        Link red 0.1,
        Joint red 0.0,
        Link red 0.1
      ]
      (1.5, 0)
      [2.094,2.094],
      createPlanPntCase
      1.5
      [ Link red 0.1,
        Joint red 0.0,
        Link red 0.1,
        Joint red 0.0,
        Link red 0.1
      ]
      (1.5, 0)
      [2.094,2.094]
  ]

-- | Executes testPlanPnt
executePlanPntTestCases :: IO Bool
executePlanPntTestCases = foldM f True planPntTestCases
  where
    f True params = testPlanPnt params
    f False _ = return False


-- | Create a Test Case for stigPlanSeg
createPlanSegCase :: Float -> Arm -> (Float, Float) -> (Float, Float) -> Motion -> (Float, Arm, LineSegment 2 () Float, Motion)
createPlanSegCase f a (xP0, yP0) (xP1, yP1) m = (f, a, ClosedLineSegment (Point2 xP0 yP0 :+ ()) (Point2 xP1 yP1 :+ ()), m)

-- | Test stigPlanSeg
testPlanSeg :: (Float, Arm, LineSegment 2 () Float, Motion) -> IO Bool
testPlanSeg (f, arm, sT, mT)
  = 
    do
      -- Expected Position
      let pT = sT ^. (end . core)
      let qT = motionToJointVector mT
      let xTargetGlobal = pointToHomogenousPoint pT

      -- Arm
      let (a, _) = getArmKinematicAndMotion f arm

      -- Calculate Answer
      mB <- stigPlanSeg f arm sT
      if null mB then
        return $ bool (trace "Wrong Null" False) (null mT) (null mT)
      else
        do
          let qB = motionToJointVector mB

          -- Forward Transforms
          let fwdTT = applyForwardKinematicTrans a qT
          let fwdTB = applyForwardKinematicTrans a qB

          -- Bat Global Position
          let batGlobalB = applyForwardKinematicMatrixTrans fwdTB Numerical.#> homogeneousZero
          let batGlobalT = applyForwardKinematicMatrixTrans fwdTT Numerical.#> homogeneousZero

          -- Error
          let eB = xTargetGlobal - batGlobalB
          let eNormB = Numerical.norm_2 eB

          let eT = trace ("Best " ++ show (batGlobalB, eNormB)) $ xTargetGlobal - batGlobalT
          let eNormT = Numerical.norm_2 eT

          let result = trace ("Target " ++ show (batGlobalT, eNormT, mT)) $
                (globalThreshold 0 eNormB == 0)
                  && ((eNormB <= eNormT) || (eNormT > 0 && eNormB / eNormT <= 1.1) || (eNormT == 0 && globalThreshold 0 eNormB == 0))
          return $ result && bool (trace "Different Motion Sizes" False) True (length mB == length mT)

-- | Test Cases for testPlanPnt
planSegTestCases :: [(Float, Arm, LineSegment 2 () Float, Motion)]
planSegTestCases =
  [ createPlanSegCase
      1.5
      [ Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.1
      ]
      (1.48023, 0.79501)
      (1.48023, 0.89501)
      [0.2, -0.2, -0.1, 0.1],
    createPlanSegCase
      1.5
      [ Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.2,
        Joint red 0.0,
        Link red 0.1
      ]
      (1.71174, 0.75003)
      (1.66379, 0.83779)
      [-0.5, 0.0, 0.4, 0.6],
    createPlanSegCase
      1.5
      [ Link red 0.1,
        Joint red 0.0,
        Link red 0.1,
        Joint red 0.0,
        Link red 0.1
      ]
      (1.4134, 5.0e-2)
      (1.5, 0)
      [2.094,2.094]
  ]


-- | Executes testPlanSeg
executePlanSegTestCases :: IO Bool
executePlanSegTestCases = foldM f True planSegTestCases
  where
    f True params = testPlanSeg params
    f False _ = return False
 
