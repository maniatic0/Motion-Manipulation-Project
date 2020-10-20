{-# LANGUAGE ScopedTypeVariables #-}

-- By Christian Oliveros and Minmin Chen
module PingPong.Player.Stig (stig) where

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

-- | Stig's player
stig :: Player
stig =
  Player
    { name = "Stig",
      arm = stigArm,
      foot = stigFoot,
      prepare = return (),
      action = stigAction,
      collide = stigCollide,
      planPnt = stigPlanPnt,
      planSeg = stigPlanSeg
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

-- | Separation from the center of the table
stigFoot :: Float
stigFoot = 1.2

-- | Stig rest postion
stigRest :: Motion
stigRest = getCurrentJoints stigArm

-- | Get the a zeroed Motion list for Stig's arm
stigNoMotion :: Motion
stigNoMotion = map f stigRest
  where
    f = const 0

-- | Calculate Motion to Rest Position. !Warning: no limits are applied
armToStigRestMotion :: Arm -> Motion
armToStigRestMotion ar = zipWith f stigRest $ getCurrentJoints ar
  where
    g = globalThreshold 0.0
    f = deltaAngle . g

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
stigAction t (tColl, Air) bs arm =
  return $
    -- Initial state, ball is falling towards some player
    let xdir = view xComponent $ dir bs
        toRest = armToStigRestMotion arm
        motion = bool stigNoMotion toRest (xdir > 0) -- xDir > 0 -> towards us
     in applyMotionLimits motion -- Velocity limits
stigAction t (tColl, coll) bs arm =
  return $
    let xdir = view xComponent $ dir bs
        toRest = armToStigRestMotion arm
        motion = bool [1, -1, 1, -1] toRest (xdir > 0)
     in applyMotionLimits motion -- Velocity limits

-- | Stig Plan Threshold
stigPlanThreshold :: (Num r, Ord r, Fractional r) => r -> r -> r
stigPlanThreshold = threshold 0.01

-- | Calculates the possible motion values to achieve a point. If it fails it returns []
stigPlanPnt :: Float -> Arm -> Point 2 Float -> IO Motion
stigPlanPnt foot arm p
  | stigPlanThreshold 0 eB == 0 = return $ map normalizeAngle $ jointVectorToMotion qB
  | otherwise = return []
  where
    (a, m) = getArmKinematicAndMotion foot arm
    q = motionToJointVector m
    (qB, eB) = newtonRaphsonIK a (homogeneousZero, pointToHomogenousPoint p) q

-- | Calculates the possible motion values to achieve a line segment. If it fails it returns []
stigPlanSeg :: Float -> Arm -> LineSegment 2 () Float -> IO Motion
stigPlanSeg foot arm s
  | isOnlyBat = return [] -- No idea if we reach it because we can't move
  | isOnlyBatJoint = return $ bool [] onlyBatJointQ onlyBatJointCheck -- If we can rotate the only useful joint to match the end point
  | normalCheck = return $ map normalizeAngle qF
  | otherwise = return []
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

-- | Create a Test Case for stigPlanPnt
createPlanPntCase :: Float -> Arm -> (Float, Float) -> Motion -> (Float, Arm, Point 2 Float, Motion)
createPlanPntCase f a (xT, yT) m = (f, a, Point2 xT yT, m)

{- -- | Test stigPlanPnt
testPlanPnt :: (Float, Arm, Point 2 Float, Motion) -> IO Bool
testPlanPnt (f, arm, pT, mT)
  | null mT = null mB
  | null mB = null mT
  | otherwise = result && (length mB == length mT)
  where
    -- Expected Position
    qT = motionToJointVector mT
    xTargetGlobal = pointToHomogenousPoint pT

    -- Arm
    (a, _) = getArmKinematicAndMotion f arm

    -- Calculate Answer
    mB = stigPlanPnt f arm pT
    qB = motionToJointVector mB

    -- Forward Transforms
    fwdTT = applyForwardKinematicTrans a qT
    fwdTB = applyForwardKinematicTrans a qB

    -- Bat Global Position
    batGlobalB = applyForwardKinematicMatrixTrans fwdTB Numerical.#> homogeneousZero
    batGlobalT = applyForwardKinematicMatrixTrans fwdTT Numerical.#> homogeneousZero

    -- Error
    eB = xTargetGlobal - batGlobalB
    eNormB = Numerical.norm_2 eB

    eT = trace ("Best " ++ show (batGlobalB, eNormB)) $ xTargetGlobal - batGlobalT
    eNormT = Numerical.norm_2 eT

    result = trace ("Target " ++ show (batGlobalT, eNormT)) $ (globalThreshold 0 eNormB == 0) && (eNormB <= eNormT)
 -}
-- | Test Cases for testPlanPnt
planPntTestCases :: [(Float, Arm, Point 2 Float, Motion)]
planPntTestCases =
  [ createPlanPntCase
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
      [0.1, -0.2, 0.3, -0.4]
  ]

{- -- | Executes testPlanPnt
executePlanPntTestCases :: Bool
executePlanPntTestCases = all testPlanPnt planPntTestCases
 -}

-- | Create a Test Case for stigPlanSeg
createPlanSegCase :: Float -> Arm -> (Float, Float) -> (Float, Float) -> Motion -> (Float, Arm, LineSegment 2 () Float, Motion)
createPlanSegCase f a (xP0, yP0) (xP1, yP1) m = (f, a, ClosedLineSegment (Point2 xP0 yP0 :+ ()) (Point2 xP1 yP1 :+ ()), m)

{- -- | Test stigPlanSeg
testPlanSeg :: (Float, Arm, LineSegment 2 () Float, Motion) -> Bool
testPlanSeg (f, arm, sT, mT)
  | null mT = null mB
  | null mB = null mT
  | otherwise = result && bool (trace "Different Motion Sizes" False) True (length mB == length mT)
  where
    -- Expected Position
    pT = sT ^. (end . core)
    qT = motionToJointVector mT
    xTargetGlobal = pointToHomogenousPoint pT

    -- Arm
    (a, _) = getArmKinematicAndMotion f arm

    -- Calculate Answer
    mB = stigPlanSeg f arm sT
    qB = motionToJointVector mB

    -- Forward Transforms
    fwdTT = applyForwardKinematicTrans a qT
    fwdTB = applyForwardKinematicTrans a qB

    -- Bat Global Position
    batGlobalB = applyForwardKinematicMatrixTrans fwdTB Numerical.#> homogeneousZero
    batGlobalT = applyForwardKinematicMatrixTrans fwdTT Numerical.#> homogeneousZero

    -- Error
    eB = xTargetGlobal - batGlobalB
    eNormB = Numerical.norm_2 eB

    eT = trace ("Best " ++ show (batGlobalB, eNormB)) $ xTargetGlobal - batGlobalT
    eNormT = Numerical.norm_2 eT

    result =
      trace ("Target " ++ show (batGlobalT, eNormT)) $
        (globalThreshold 0 eNormB == 0)
          && ((eNormB <= eNormT) || (eNormT > 0 && eNormB / eNormT <= 1.1) || (eNormT == 0 && globalThreshold 0 eNormB == 0))
 -}

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
      [-0.5, 0.0, 0.4, 0.6]
  ]

{-
-- | Executes testPlanSeg
executePlanSegTestCases :: Bool
executePlanSegTestCases = all testPlanSeg planSegTestCases
 -}
