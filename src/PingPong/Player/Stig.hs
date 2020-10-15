{-# LANGUAGE ScopedTypeVariables #-}

-- By Christian Oliveros and Minmin Chen
module PingPong.Player.Stig where --(stig)

import Control.Lens (view, (&), (.~), (^.))
import Data.Bool (bool)
import Data.Ext
import Data.Fixed (mod')
import Data.Geometry hiding (head)
import Data.Geometry.Vector.VectorFamilyPeano
import Data.Maybe
import Debug.Trace
import GHC.Float
import Graphics.Gloss (Color, makeColor)
import qualified Numeric.LinearAlgebra as Numerical
import PingPong.Model
import PingPong.Player

-- Geometry Helpers

-- | Threshold function
threshold :: (Num r, Ord r) => r -> r -> r -> r
threshold limit target val
  | abs diff < limit = target
  | otherwise = val
  where
    diff = target - val

-- | Common Threshold for functions
globalThreshold :: (Num r, Ord r, Fractional r) => r -> r -> r
globalThreshold = threshold 0.001

-- | tau = 2 * pi
tau :: forall a. Floating a => a
tau = 2 * pi

-- | Normalize Angle in Radians between (-pi, pi]
-- Based on https://stackoverflow.com/a/2323034
normalizeAngle :: (Num r, Ord r, Floating r, Fractional r, Real r) => r -> r
normalizeAngle a =
  let a1 = mod' a tau
      a2 = mod' (a + tau) tau
   in bool a2 (a2 - tau) (a2 > pi)

-- | Shortest Distance Between Two Angles in Radians [-pi, pi)
deltaAngle :: (Num r, Ord r, Floating r, Fractional r, Real r) => r -> r -> r
deltaAngle a1 a2
  | diff >= pi = diff - tau
  | diff < - pi = tau + diff
  | otherwise = diff
  where
    a1N = normalizeAngle a1
    a2N = normalizeAngle a2
    diff = a1N - a2N

-- | 2D outer product (otherwise known as 2D cross product)
outer2D :: (Num r) => Vector 2 r -> Vector 2 r -> r
outer2D v1 v2 = res
  where
    x1 = view xComponent v1
    y1 = view yComponent v1
    x2 = view xComponent v2
    y2 = view yComponent v2
    res = (x1 * y2) - (y1 * x2)

-- | Solve a x^2 + b * x + c. It can return more than one solution and the [] solution means that all x are valid
solveQuadratic :: (Num r, Eq r, Ord r, Fractional r, Floating r) => r -> r -> r -> Maybe [r]
solveQuadratic a b c
  | isAZero && isBZero =
    if isCZero
      then Just []
      else Nothing
  | isAZero && not isBZero = Just [- c / b]
  | not isAZero = if isDeltaAboveOrZero then res else Nothing
  where
    -- Checks
    isAZero = globalThreshold 0 a == 0
    isBZero = globalThreshold 0 b == 0
    isCZero = globalThreshold 0 c == 0
    -- Discriminant from the Quadratic Formula
    delta = (b * b) - (4 * a * c)
    -- Check if zero
    deltaThreshold = globalThreshold 0 delta
    -- Check above or  equal zero
    isDeltaAboveOrZero = deltaThreshold >= 0
    -- Rest of the Quadratic Formula
    x1 = (- b + sqrt deltaThreshold) / (2 * a)
    x2 = (- b - sqrt deltaThreshold) / (2 * a)
    res = Just [x1, x2]

-- | Obtain the factor of a point projection to a line segment, normalized by the line segment lentgh. To be Correct it has to be between 0 and 1
-- Not sure how to generalize dimension
pointLineSegmentProjectionNormalizedFactor :: (Num r, Floating r, Ord r) => Point 2 r -> LineSegment 2 () r -> r
pointLineSegmentProjectionNormalizedFactor p l
  | isDegenerate = 0
  | otherwise = res
  where
    startPoint = l ^. (start . core . vector)
    endPoint = l ^. (end . core . vector)
    w = endPoint ^-^ startPoint
    wNorm = globalThreshold 0 $ norm w -- Included Threshold 0
    isDegenerate = wNorm == 0
    c = p ^. vector
    res = ((c ^-^ startPoint) `dot` w) / (wNorm * wNorm)

-- | Checks if a point was intersected by a moving line. It returns the time of intersection
pointMovingLineInterception :: (Num r, Floating r, Ord r) => Point 2 r -> LineSegment 2 () r -> LineSegment 2 () r -> Maybe r
pointMovingLineInterception p l0 l1 =
  case possible of
    Nothing -> Nothing -- No solution
    Just _ -> res
  where
    p0I = l0 ^. (start . core . vector)
    p0F = l1 ^. (start . core . vector)
    p0d = p0F ^-^ p0I
    p1I = l0 ^. (end . core . vector)
    p1F = l1 ^. (end . core . vector)
    p1d = p1F ^-^ p1I
    c = p ^. vector
    w0 = c ^-^ p0I
    w1 = p1I ^-^ p0I
    w2 = p1d ^-^ p0d
    a0 = outer2D w0 w1 -- w0 x w1
    a1 = outer2D w1 p0d + outer2D w0 w2 -- w1 x p0d + w0 x w2
    a2 = outer2D w2 p0d -- w2 x p0d
    possible = solveQuadratic a2 a1 a0
    tsRaw = fromJust possible
    -- If solveQuadratic is [] change for 0, because all t are valid
    -- Also, make sure the times are valid between 0 and 1 (threshold is used for approximations)
    ts = filter (\t -> 0 <= t && t <= 1 && inLine t) $ map (globalThreshold 1 . globalThreshold 0) $ bool tsRaw [0] (null tsRaw)
    inLine t = 0 <= a && a <= 1
      where
        p0 = lerp t p0F p0I
        p1 = lerp t p1F p1I
        a = pointLineSegmentProjectionNormalizedFactor p (ClosedLineSegment ((origin & vector .~ p0) :+ ()) ((origin & vector .~ p1) :+ ()))
    res = case ts of
      [] -> Nothing -- No solution
      _ -> Just $ minimum ts -- Minimum Valid Time

-- | Move a line by a vector
moveLineByVector :: (Num r) => LineSegment 2 () r -> Vector 2 r -> LineSegment 2 () r
moveLineByVector l v = res
  where
    p0I = l ^. (start . core . vector)
    p0F = l ^. (end . core . vector)
    p1I = p0I ^+^ v
    p1F = p0F ^+^ v
    res = ClosedLineSegment ((origin & vector .~ p1I) :+ ()) ((origin & vector .~ p1F) :+ ())

-- | Interception Info
data InterceptionInfo r = InterceptionInfo
  { -- | Time of Interception
    interptTime :: r,
    -- | Point where interception happened
    point :: Point 2 r
  }
  deriving (Show)

-- | Check for interception between a moving point and a moving line
movingPointMovingLineInterception :: (Num r, Floating r, Ord r) => (Point 2 r, LineSegment 2 () r) -> (Point 2 r, LineSegment 2 () r) -> Maybe (InterceptionInfo r)
movingPointMovingLineInterception (p0, l0) (p1, l1) =
  case tRaw of
    Nothing -> Nothing -- No Collision
    Just _ -> res
  where
    pI = p0 ^. vector
    pF = p1 ^. vector
    pd = pF ^-^ pI
    lF = moveLineByVector l1 (negated pd)
    tRaw = pointMovingLineInterception p0 l0 lF
    t = fromJust tRaw
    c = lerp t pF pI
    res = Just $ InterceptionInfo t (origin & vector .~ c)

-- | Reflect a direction vector againt a normal vector (it must be normalized)
reflect :: (Num r, Floating r, Ord r, Show r) => Vector 2 r -> Vector 2 r -> Vector 2 r
reflect d n
  | normCheck == 1 || normCheck == 0 = r
  | otherwise = error ("Normal is not zero or normalized: " ++ show n ++ " with norm=" ++ show normCheck)
  where
    r = n ^* (2 * dot d n) ^-^ d
    normCheck = globalThreshold 0 $ globalThreshold 1 (norm n)

-- | Normalize a vector and get its norm
normalizeVector :: (Num r, Floating r, Ord r) => Vector 2 r -> (Vector 2 r, r)
normalizeVector v = (n, len)
  where
    len = globalThreshold 0 $ norm v
    n = bool (Vector2 0 0) (v ^* (1 / len)) (len > 0)

-- | Calculates a linear collision between a point and line
movingBallMovingLineCollide ::
  (Num r, Floating r, Ord r, Show r) =>
  (r, Point 2 r, LineSegment 2 () r) ->
  (r, Point 2 r, LineSegment 2 () r) ->
  Point 2 r
movingBallMovingLineCollide (t0, p0, l0) (t1, p1, l1)
  | dt <= 0 = p1 -- Negative time or dt = 0
  | otherwise = case interceptionMaybe of
    Nothing -> p1
    Just _ -> pCol
  where
    dt = globalThreshold 0 $ t1 - t0
    invDt = 1 / dt
    -- Interception Info
    interceptionMaybe = movingPointMovingLineInterception (p0, l0) (p1, l1)
    interception = fromJust interceptionMaybe
    tc = interptTime interception
    tcScaled = t0 + dt * tc -- To get it back into scale
    pc = point interception
    -- Point Info
    pI = p0 ^. vector
    pF = p1 ^. vector
    pd = pF ^-^ pI
    pv = pd ^* invDt
    -- Line Start Point Info
    p0I = l0 ^. (start . core . vector)
    p0F = l1 ^. (start . core . vector)
    p0d = p0F ^-^ p0I
    p0v = p0d ^* invDt
    -- Line End Point Info
    p1I = l0 ^. (end . core . vector)
    p1F = l1 ^. (end . core . vector)
    p1d = p1F ^-^ p1I
    p1v = p1d ^* invDt
    -- Line at Collision
    p0c = lerp tc p0F p0I
    p1c = lerp tc p1F p1I
    a = pointLineSegmentProjectionNormalizedFactor pc (ClosedLineSegment ((origin & vector .~ p0c) :+ ()) ((origin & vector .~ p1c) :+ ()))
    lcv = lerp a p1v p0v -- Interpolated line velocity
    lcDir = p1c ^-^ p0c
    (lcDirNormalized, lcNorm) = normalizeVector lcDir
    -- Collision
    --halfVectorUnNormalized = pv ^+^ lcv
    --(halfVector, halfNorm) = normalizeVector halfVectorUnNormalized
    vd = pv ^-^ lcv -- Difference of velocities
    --rVd = reflect vd $ bool halfVector lcDirNormalized (lcNorm > 0) -- Use line as normal for reflection if possible. In case the line degenerated to a point use HalfVector
    rVd = reflect vd lcDirNormalized -- Use line as normal for reflection if possible. In case the line degenerated to a point use HalfVector
    cvd = rVd ^+^ lcv -- Collision Velocity Direction
    pCol = pc .+^ (cvd ^* (t1 - tcScaled))

-- | Calculate the forward kinematic matrix for a joint / link
calFwdMatrix :: Element -> Element -> [[Float]]
calFwdMatrix (Joint _ a) (Link _ l) =
  [ [cos a, - sin a, l * cos a],
    [sin a, cos a, l * sin a],
    [0, 0, 1]
  ]
calFwdMatrix _ _ = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

-- End of Geometry Helpers

-- Simulation Helpers

-- | Check that the arm elements are valid
checkArmElements :: Arm -> Arm
checkArmElements [] = error "The Empty Arm is no valid"
checkArmElements l@[Link _ len] = bool (error "Bat has wrong length it must be 0.1") l (len == 0.1)
checkArmElements [Joint _ _] = error "A Joint can't be the end of the Arm"
checkArmElements (j@(Joint _ _) : xs@(Link _ _ : _)) = j : checkArmElements xs
checkArmElements (Joint _ _ : (Joint _ _ : _)) = error "Two Joints together"
checkArmElements (Link _ _ : (Link _ _ : _)) = error "Two Links together"
checkArmElements (l@(Link _ len) : xs@(Joint _ _ : _)) =
  bool (error "Arm has a Link with wrong length [.1, .5]") (l : checkArmElements xs) (0.1 <= len && len <= 0.5)

-- | Check that the arm is valid
checkArm :: Arm -> Arm
checkArm ar =
  bool (error "Wrong number of Joints [2..5]") (checkArmElements ar) (2 <= jointCount && jointCount <= 5)
  where
    jointCount = foldl f 0 ar
      where
        f :: Integer -> Element -> Integer
        f n (Joint _ _) = n + 1
        f n (Link _ _) = n

-- | Get the current Joint parameters as a Motion
getCurrentJoints :: Arm -> Motion
getCurrentJoints [] = []
getCurrentJoints (Joint _ rad : xs) = rad : getCurrentJoints xs
getCurrentJoints (Link _ _ : xs) = getCurrentJoints xs

-- | Motion Velocity Limit
motionLimit :: Float
motionLimit = 1.0

-- | Apply Motion limits to avoid simulation problems
applyMotionLimits :: Motion -> Motion
applyMotionLimits = map f
  where
    f x = min motionLimit $ max (- motionLimit) x

-- | Simplify arm with adjacent joints or links
simplifyArm :: Arm -> Arm
--simplifyArm (Joint color a1 : (Joint _ a2 : xs)) = (Joint color (a1 + a2)) : xs
simplifyArm [] = []
simplifyArm (Link color l1 : Link _ l2 : xs) = Link color (l1 + l2) : xs
simplifyArm (x : xs) = x : simplifyArm xs

-- End of Simulation Helpers

-- | Stig's player
stig :: Player
stig =
  Player
    { name = "Stig",
      arm = stigArm,
      foot = stigFoot,
      action = stigAction,
      collide = stigCollide,
      planPnt = stigPlanPnt,
      planSeg = stigPlanSeg
    }

paleBlue :: Color
paleBlue = makeColor 0.5 0.5 0.6 1

red :: Color
red = makeColor 1 0 0 1

hotPink :: Color
hotPink = makeColor 1.0 0.2 0.7 1

-- | Arm to use
stigArm :: Arm
stigArm =
  checkArm
    [ Joint red (-0.3), -- (0.1)
      Link paleBlue 0.5,
      Joint red 1.3, -- (0.1)
      Link paleBlue 0.4,
      Joint red 0.9, -- (-0.1)
      Link paleBlue 0.2,
      Joint red 0.5, -- (-0.1)
      Link hotPink 0.1 -- Bat
    ]

-- | Separation from the center of the table
stigFoot :: Float
stigFoot = 1.3

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

stigCollide ::
  forall r.
  (Num r, Floating r, Ord r, Eq r, Show r) =>
  (r, Point 2 r, LineSegment 2 () r) ->
  (r, Point 2 r, LineSegment 2 () r) ->
  Point 2 r
stigCollide = bool (error "Stig Collide Failed a Test Case") f completeCheck
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

stigAction :: BallState -> Arm -> IO Motion
stigAction bs arm =
  return $
    let xdir = view xComponent $ dir bs
        toRest = armToStigRestMotion arm
        motion = bool [1, -1, 1, -1] toRest (xdir > 0)
     in applyMotionLimits motion -- Velocity limits

-- Inverse and Forward Kinematics

data ArmKinematicPart
  = ArmBase
      { footParam :: Numerical.R
      }
  | ArmJoint
      { translationParam :: Numerical.R
      }
  | ArmBat
      { translationParam :: Numerical.R
      }
  deriving (Show)

type ArmKinematic = [ArmKinematicPart]

-- | Goes from rough representation to a more refined one
fromFootArmToArmKinematic :: Float -> Arm -> ArmKinematic
fromFootArmToArmKinematic foot arm = ArmBase (float2Double foot) : processArm 0 arm
  where
    processArm :: Numerical.R -> Arm -> ArmKinematic
    processArm t [Link _ t2] = [ArmBat $ t + float2Double t2]
    processArm t (Link _ t2 : as) = processArm (t + float2Double t2) as
    processArm t (Joint _ _ : as) = ArmJoint t : processArm 0 as

-- | Goes from rough representation to a more refined one.
-- As well, get the motion vector
getArmKinematicAndMotion :: Float -> Arm -> (ArmKinematic, Motion)
getArmKinematicAndMotion foot arm =
  (fromFootArmToArmKinematic foot simpleArm, getCurrentJoints simpleArm)
  where
    simpleArm = simplifyArm arm

-- | Converts a motion list to a vector of joint coordinates
motionToJointVector :: Motion -> Numerical.Vector Numerical.R
motionToJointVector m = Numerical.fromList $ map float2Double m

-- | Converts vector of joint coordinates to a motion list
jointVectorToMotion :: Numerical.Vector Numerical.R -> Motion
jointVectorToMotion v = map double2Float $ Numerical.toList v

-- | Translation in the X axis matrix
translateXTrans :: Numerical.R -> Numerical.Matrix Numerical.R
translateXTrans t =
  Numerical.fromLists [[1, 0, t], [0, 1, 0], [0, 0, 1]] :: Numerical.Matrix Numerical.R

-- | Translation in the X axis inverse matrix
translateXInvTrans :: Numerical.R -> Numerical.Matrix Numerical.R
translateXInvTrans t =
  Numerical.fromLists [[1, 0, - t], [0, 1, 0], [0, 0, 1]] :: Numerical.Matrix Numerical.R

-- | Rotation matrix
rotateTrans :: Numerical.R -> Numerical.Matrix Numerical.R
rotateTrans a =
  Numerical.fromLists [[c, - s, 0], [s, c, 0], [0, 0, 1]] :: Numerical.Matrix Numerical.R
  where
    c = cos a
    s = sin a

-- | Rotation inverse matrix
rotateInvTrans :: Numerical.R -> Numerical.Matrix Numerical.R
rotateInvTrans a =
  Numerical.fromLists [[c, s, 0], [- s, c, 0], [0, 0, 1]] :: Numerical.Matrix Numerical.R
  where
    c = cos a
    s = sin a

-- | Create an homogeneous 2D point
homogeneousPoint :: Numerical.R -> Numerical.R -> Numerical.Vector Numerical.R
homogeneousPoint x y = Numerical.fromList [x, y, 1]

-- | Zero Point in Homogenous Coordinates
homogeneousZero :: Numerical.Vector Numerical.R
homogeneousZero = homogeneousPoint 0 0

-- | 2d Point to Homogeneous Coordinates
pointToHomogenousPoint :: Point 2 Float -> Numerical.Vector Numerical.R
pointToHomogenousPoint p = homogeneousPoint (float2Double $ view xCoord p) (float2Double $ view yCoord p)

-- | Create an homogeneous 2D vector
homogeneousVector :: Numerical.R -> Numerical.R -> Numerical.Vector Numerical.R
homogeneousVector x y = Numerical.fromList [x, y, 0]

-- | X Vector
homogeneousVectorX :: Numerical.Vector Numerical.R
homogeneousVectorX = homogeneousVector 1 0

-- | Y Vector
homogeneousVectorY :: Numerical.Vector Numerical.R
homogeneousVectorY = homogeneousVector 0 1

-- | Homogeneous 2D Identity Matrix
homogeneousIdent :: Numerical.Matrix Numerical.R
homogeneousIdent = Numerical.ident 3

data ArmKinematicPartMatrix
  = ArmBaseMatrix
      { footMatrix :: Numerical.Matrix Numerical.R
      }
  | ArmJointMatrix
      { translationMatrix :: Numerical.Matrix Numerical.R,
        jointMatrix :: Numerical.Matrix Numerical.R
      }
  | ArmBatMatrix
      { batMatrix :: Numerical.Matrix Numerical.R
      }
  deriving (Show)

type ArmKinematicMatrix = [ArmKinematicPartMatrix]

-- | Get Transform from Part Matrix
getTrans :: ArmKinematicPartMatrix -> Numerical.Matrix Numerical.R
getTrans (ArmBaseMatrix t) = t
getTrans (ArmJointMatrix t r) = t <> r
getTrans (ArmBatMatrix t) = t

-- | Get Inverse Transform from Part Matrix
getTransInv :: ArmKinematicPartMatrix -> Numerical.Matrix Numerical.R
getTransInv (ArmBaseMatrix t) = t
getTransInv (ArmJointMatrix t r) = r <> t
getTransInv (ArmBatMatrix t) = t

-- | Calculate the transforms for forward kinematics
applyForwardKinematicTrans ::
  ArmKinematic -> Numerical.Vector Numerical.R -> ArmKinematicMatrix
applyForwardKinematicTrans arm v = toTrans arm motion
  where
    motion = Numerical.toList v :: [Numerical.R]

    toTrans :: ArmKinematic -> [Numerical.R] -> ArmKinematicMatrix
    toTrans (ArmBase t : as) js = (ArmBaseMatrix $ translateXTrans t <> rotateTrans (pi / 2)) : toTrans as js
    toTrans (ArmJoint t : as) (j : js) = ArmJointMatrix (translateXTrans t) (rotateTrans j) : toTrans as js
    toTrans [ArmBat t] [] = [ArmBatMatrix $ translateXTrans t]

-- | Calculate the inverse transforms for forward kinematics
applyForwardKinematicTransInv ::
  ArmKinematic -> Numerical.Vector Numerical.R -> ArmKinematicMatrix
applyForwardKinematicTransInv arm v = reverse $ toTrans arm motion
  where
    motion = Numerical.toList v :: [Numerical.R]

    toTrans :: ArmKinematic -> [Numerical.R] -> ArmKinematicMatrix
    toTrans (ArmBase t : as) js = (ArmBaseMatrix $ rotateInvTrans (pi / 2) <> translateXInvTrans t) : toTrans as js
    toTrans (ArmJoint t : as) (j : js) = ArmJointMatrix (translateXInvTrans t) (rotateInvTrans j) : toTrans as js
    toTrans [ArmBat t] [] = [ArmBatMatrix $ translateXInvTrans t]

-- | Apply Forward Kinematic Transformations
applyForwardKinematicMatrixTrans :: ArmKinematicMatrix -> Numerical.Matrix Numerical.R
applyForwardKinematicMatrixTrans = foldr ((<>) . getTrans) homogeneousIdent

-- | Apply Forward Kinematic Inverse Transformations (the list must go from end effecto to base)
applyForwardKinematicMatrixTransInv :: ArmKinematicMatrix -> Numerical.Matrix Numerical.R
applyForwardKinematicMatrixTransInv = foldr ((<>) . getTransInv) homogeneousIdent

-- | Compress the transforms of a Forward Kinematic Arm to only the joints and the end effector
compressForwardKinematicsJointsTrans :: ArmKinematicMatrix -> ArmKinematicMatrix
compressForwardKinematicsJointsTrans a = compress a homogeneousIdent
  where
    compress :: ArmKinematicMatrix -> Numerical.Matrix Numerical.R -> ArmKinematicMatrix
    compress [ArmBatMatrix t] m = [ArmBatMatrix (m <> t)]
    compress (ArmJointMatrix t r : as) m = ArmJointMatrix (m <> t) r : compress as homogeneousIdent
    compress (a : as) m = compress as (m <> getTrans a)

-- | Compress the transforms of a Forward Kinematic Arm to only the joints and the end base
compressForwardKinematicsJointsInvTrans :: ArmKinematicMatrix -> ArmKinematicMatrix
compressForwardKinematicsJointsInvTrans a = compress a homogeneousIdent
  where
    compress :: ArmKinematicMatrix -> Numerical.Matrix Numerical.R -> ArmKinematicMatrix
    compress [ArmBaseMatrix t] m = [ArmBaseMatrix (t <> m)]
    compress (ArmJointMatrix t r : as) m = ArmJointMatrix (t <> m) r : compress as homogeneousIdent
    compress (a : as) m = compress as (getTransInv a <> m)

-- | Calculate the 2D Homogeneous Jacobian of an arm [J^T|0]^T .
-- fwdMatTrans : Kinematic Forward Transforms (From base to end effector).
-- fwdMatTransInv : Kinematic Forward Inverse Transforms (From end effector to base).
-- xLocal : Position of Tool on End Effector Local Coordinates
calculateJacobian ::
  ArmKinematicMatrix -> ArmKinematicMatrix -> Numerical.Vector Numerical.R -> Numerical.Matrix Numerical.R
calculateJacobian fwdMatTrans fwdMatTransInv xLocal = jH
  where
    -- Compressed form guarantees that the links were applied to the joints
    fwdTransCompressed = compressForwardKinematicsJointsTrans fwdMatTrans
    fwdTransInvCompressed = compressForwardKinematicsJointsInvTrans fwdMatTransInv

    -- Derivative of a revolute Joint
    revoluteDeriv = Numerical.fromLists [[0, -1, 0], [1, 0, 0], [0, 0, 0]] :: Numerical.Matrix Numerical.R

    rollingTransInv :: ArmKinematicMatrix -> Numerical.Matrix Numerical.R -> [Numerical.Matrix Numerical.R]
    rollingTransInv [ArmBaseMatrix t] m = [t <> m]
    rollingTransInv (a : as) m = roll : rollingTransInv as roll
      where
        roll = getTransInv a <> m
    calculate ::
      ArmKinematicMatrix -> Numerical.Matrix Numerical.R -> [Numerical.Matrix Numerical.R] -> [Numerical.Vector Numerical.R]
    calculate [ArmBatMatrix _] _ [_] = [] -- We reach the bat, we no longer need to process
    calculate (j@(ArmJointMatrix _ _) : as) m (endToJ : mIs) =
      (baseToJ <> revoluteDeriv <> endToJ) Numerical.#> xLocal : calculate as baseToJ mIs
      where
        baseToJ = m <> getTrans j

    -- Jacobian in this form: [J^T|0]^T there is a row of 0s at the end due to 2d homogeneous coordinates
    jH =
      Numerical.fromColumns $
        calculate fwdTransCompressed homogeneousIdent (rollingTransInv fwdTransInvCompressed homogeneousIdent)

-- | Get the Jacobian from the Homogeneous Jacobian (Note this doesn't work for the inverse of the Jacobian)
getJacobianFromHomogeneousJacobian :: Numerical.Matrix Numerical.R -> Numerical.Matrix Numerical.R
getJacobianFromHomogeneousJacobian jH = jH Numerical.?? (Numerical.DropLast 1, Numerical.All)

-- | Newton Raphson Threshold
newtonRaphsonThreshold :: (Num r, Ord r, Fractional r) => r -> r -> r
newtonRaphsonThreshold = threshold 0.0000001

-- | Calculates a Newton-Raphson IK step (if singular it fails). Returns the new Joints and its error agains target
-- a : Arm Description.
-- q : Current Joints.
-- xLocal : Tool in End-Effector Local Coordinates
-- xTargetGlobal : Target in Global Coordinates
newtonRaphsonStep ::
  ArmKinematic ->
  Numerical.Vector Numerical.R ->
  Numerical.Vector Numerical.R ->
  Numerical.Vector Numerical.R ->
  Maybe (Numerical.Vector Numerical.R, Numerical.R)
newtonRaphsonStep a q xLocal xTargetGlobal
  | singular = Nothing
  | otherwise = Just (qn, Numerical.norm_2 eN)
  where
    -- Forward Transforms
    fwdT = applyForwardKinematicTrans a q
    fwdTI = applyForwardKinematicTransInv a q
    jH = calculateJacobian fwdT fwdTI xLocal

    -- Bat Global Position
    batGlobal = applyForwardKinematicMatrixTrans fwdT Numerical.#> xLocal

    -- Error
    e = xTargetGlobal - batGlobal

    -- Drop Homogeneous part
    er = Numerical.subVector 0 2 e
    j = getJacobianFromHomogeneousJacobian jH

    -- Calculate delta q
    dq = - Numerical.pinv j Numerical.#> er

    dqNorm = Numerical.norm_2 dq

    qnUnormalized = q + dq
    -- Re normalizes angles for more accuracy
    qn = Numerical.fromList $ map normalizeAngle $ Numerical.toList qnUnormalized

    -- New Bat Position
    batN = applyForwardKinematicMatrixTrans (applyForwardKinematicTrans a qn) Numerical.#> xLocal

    -- New Error
    eN = xTargetGlobal - batN

    -- If the move was singular
    singular = Numerical.rank jH < 2 && newtonRaphsonThreshold 0 dqNorm == 0

-- | Maximum Newton Raphson Step
newtonRaphsonIKMaxStep :: Int
newtonRaphsonIKMaxStep = 10000

-- | Really Bad Step
newtonRaphsonIKBadStep :: forall a. Floating a => a
newtonRaphsonIKBadStep = 10

-- | When to do a random Restart Newton Raphson Step
newtonRaphsonIKMaxRandomRestartStep :: Int
newtonRaphsonIKMaxRandomRestartStep = round (fromIntegral newtonRaphsonIKMaxStep / 10)

-- | NewtonRaphson Loop Iteration
newtonRaphsonIKIter ::
  Int ->
  Int ->
  ArmKinematic ->
  (Numerical.Vector Numerical.R, Numerical.Vector Numerical.R) ->
  Numerical.Vector Numerical.R ->
  (Numerical.Vector Numerical.R, Numerical.R) ->
  (Numerical.Vector Numerical.R, Numerical.R)
newtonRaphsonIKIter i j a (xLocal, xTargetGlobal) q (qBest, eBest)
  | i >= newtonRaphsonIKMaxStep = trace ("Limit " ++ show i ++ " " ++ show (qBest, eBest)) $ (qBest, eBest)
  | newtonRaphsonThreshold 0 eBest == 0 = trace ("Perfect " ++ show i ++ " " ++ show (qBest, eBest)) $ (qBest, eBest)
  | singular = trace ("Singular " ++ show i ++ " " ++ show (qBest, eBest)) $ newtonRaphsonIKIter (i + 1) 0 a (xLocal, xTargetGlobal) qR (qBest, eBest)
  | needReset = trace ("Need Reset " ++ show i ++ " " ++ show (qBest, eBest)) $ newtonRaphsonIKIter (i + 1) 0 a (xLocal, xTargetGlobal) qR (qBest, eBest)
  | eN < eBest = trace ("New Best " ++ show i ++ " " ++ show (qN, eN)) $ newtonRaphsonIKIter (i + 1) (j + 1) a (xLocal, xTargetGlobal) qN (qN, eN)
  | otherwise = trace ("Step " ++ show i ++ " " ++ show (qBest, eBest)) $ newtonRaphsonIKIter (i + 1) (j + 1) a (xLocal, xTargetGlobal) qN (qBest, eBest)
  where
    -- Random Vector
    qSize = Numerical.size q
    pseudoInt = round $ fromIntegral i ** (Numerical.dot q qBest * bool (eBest + 1) (1 / eBest) (eBest < 1 && 0 < eBest))
    qR = pi / 2 * (2 * Numerical.randomVector pseudoInt Numerical.Uniform qSize - 1)

    -- Perform the step
    step = newtonRaphsonStep a q xLocal xTargetGlobal

    singular = case step of
      Nothing -> True
      Just _ -> False

    (qN, eN) = fromJust step

    needReset = eN >= eBest * newtonRaphsonIKBadStep && j >= newtonRaphsonIKMaxRandomRestartStep

-- | Newton Raphson IK Algorithm
newtonRaphsonIK ::
  ArmKinematic ->
  (Numerical.Vector Numerical.R, Numerical.Vector Numerical.R) ->
  Numerical.Vector Numerical.R ->
  (Numerical.Vector Numerical.R, Numerical.R)
newtonRaphsonIK a (xLocal, xTargetGlobal) q = newtonRaphsonIKIter 0 0 a (xLocal, xTargetGlobal) q (q, eNorm)
  where
    -- Forward Transforms
    fwdT = applyForwardKinematicTrans a q

    -- Bat Global Position
    batGlobal = applyForwardKinematicMatrixTrans fwdT Numerical.#> xLocal

    -- Error
    e = xTargetGlobal - batGlobal
    eNorm = Numerical.norm_2 e

-- | Calculates the possible motion values to achieve a point. If it fails it returns []
stigPlanPnt :: Float -> Arm -> Point 2 Float -> Motion
stigPlanPnt foot arm p
  | globalThreshold 0 eB == 0 = trace ("Converges " ++ show (qB, eB)) $ map normalizeAngle $ jointVectorToMotion qB
  | otherwise = trace ("Failed to converge " ++ show (qB, eB)) $ []
  where
    (a, m) = getArmKinematicAndMotion foot arm
    q = motionToJointVector m
    (qB, eB) = newtonRaphsonIK a (homogeneousZero, pointToHomogenousPoint p) q

-- | Calculates the possible motion values to achieve a line segment. If it fails it returns []
stigPlanSeg :: Float -> Arm -> LineSegment 2 () Float -> Motion
stigPlanSeg foot arm s
  | isOnlyBat = trace ("Weird Only Bat Case") [] -- No idea if we reach it because we can't move
  | isOnlyBatJoint = trace ("Weird Only Bat and Joint Case") $ bool [] onlyBatJointQ onlyBatJointCheck -- If we can rotate the only useful joint to match the end point
  | normalCheck = trace ("Converges " ++ show (qF, eB)) $ map normalizeAngle $ qF
  | otherwise = trace ("Failed to converge " ++ show (qB, eB, smallBatToP1Norm, batLength)) $ []
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
    onlyBatJointBaseToP1Norm = Numerical.norm_2 onlyBatJointBaseToP1
    onlyBatJointBaseToP1Angle = atan2 (Numerical.atIndex onlyBatJointBaseToP1 1) (Numerical.atIndex onlyBatJointBaseToP1 0)
    onlyBatJointBaseToYAngle = pi/2
    
    onlyBatJointAngle = deltaAngle onlyBatJointBaseToP1Angle onlyBatJointBaseToYAngle
    onlyBatJointQ = reverse $ setFirstJointRest0 onlyBatJointAngle firstJointsRev
    
    -- Check that the bat has the same length as the distance from base to endpoint
    onlyBatJointCheck = globalThreshold 0 (float2Double batLength - onlyBatJointBaseToP1Norm) == 0
    
    -- From here we are in a normal case we have at least this form: link (bat) -- joint -- link -- base
    -- Small Arm must reach the start point of segment
    (a, m) = getArmKinematicAndMotion foot smallArm
    q = motionToJointVector m
    (qB, eB) = newtonRaphsonIK a (homogeneousZero, p0) q

    -- Small Arm Forward Transforms
    fwdTB = applyForwardKinematicTrans a qB
    fwdTBMat = applyForwardKinematicMatrixTrans fwdTB

    -- Small Bat Global Position
    smallBatGlobal = fwdTBMat Numerical.#> homogeneousZero
    smallBatGlobalXAxis = fwdTBMat Numerical.#> homogeneousVectorX
    smallBatGlobalXAxisNorm = smallBatGlobalXAxis / Numerical.scalar(Numerical.norm_2 smallBatGlobalXAxis)

    -- Vector from small bat to target
    smallBatToP1 = p1 - smallBatGlobal
    smallBatToP1Norm = Numerical.norm_2 smallBatToP1
    smallBatToP1Angle = atan2 (Numerical.atIndex smallBatToP1 1) (Numerical.atIndex smallBatToP1 0)
    smallBatToXAxisAngle = atan2 (Numerical.atIndex smallBatGlobalXAxisNorm 1) (Numerical.atIndex smallBatGlobalXAxisNorm 0)
    jointAngle = deltaAngle smallBatToP1Angle smallBatToXAxisAngle

    qF = m ++ reverse (setFirstJointRest0 jointAngle firstJointsRev)

    -- Check that small bat is at p0 and that we can reach p1
    normalCheck = globalThreshold 0 eB == 0 && (globalThreshold 0 (float2Double batLength - smallBatToP1Norm) == 0)

    -- | If an Element is a joint
    isJoint :: Element -> Bool
    isJoint (Joint _ _) = True
    isJoint _ = False

    -- | Get Link Length
    getLinkLength :: Element -> Float
    getLinkLength (Link _ t) = t

    -- | Set first joint with a value and the rest is 0 (the arm includes the joint)
    setFirstJointRest0 :: Double -> Arm -> Motion 
    setFirstJointRest0 q as = double2Float q : map (const 0) (tail as)
    

-- | Create a Test Case for stigPlanPnt
createPlanPntCase :: Float -> Arm -> (Float, Float) -> Motion -> (Float, Arm, Point 2 Float, Motion)
createPlanPntCase f a (xT, yT) m = (f, a, Point2 xT yT, m)

-- | Test stigPlanPnt
testPlanPnt :: (Float, Arm, Point 2 Float, Motion) -> Bool
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

-- | Test Cases for testPlanPnt
planPntTestCases :: [(Float, Arm, Point 2 Float, Motion)]
planPntTestCases =
  [
    createPlanPntCase 1.5
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
      [0.1, 0.2, 0.3, 0.4]
    ,
    createPlanPntCase 1.5
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
      [-0.5, 0.0, 0.4, 0.6]
    ,
    createPlanPntCase 1.5
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

-- | Executes testPlanPnt
executePlanPntTestCases :: Bool
executePlanPntTestCases = all testPlanPnt planPntTestCases

-- | Create a Test Case for stigPlanSeg
createPlanSegCase :: Float -> Arm -> (Float, Float) -> (Float, Float) -> Motion -> (Float, Arm, LineSegment 2 () Float, Motion)
createPlanSegCase f a (xP0, yP0) (xP1, yP1) m = (f, a, ClosedLineSegment (Point2 xP0 yP0 :+ ()) (Point2 xP1 yP1 :+ ()), m)

-- | Test stigPlanSeg
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

    result = trace ("Target " ++ show (batGlobalT, eNormT)) $ (globalThreshold 0 eNormB == 0) && (eNormB <= eNormT)

-- | Test Cases for testPlanPnt
planSegTestCases :: [(Float, Arm,LineSegment 2 () Float, Motion)]
planSegTestCases = 
  [
   {-  createPlanSegCase 1.5
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
      (1.48023, 0.79501) (1.48023, 0.89501)
      [0.2, -0.2, -0.1, 0.1]
    , -}
    createPlanSegCase 1.5
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
      (1.71174, 0.75003) (1.66379, 0.83779)
      [-0.5, 0.0, 0.4, 0.6]
  ]

-- | Executes testPlanSeg
executePlanSegTestCases :: Bool
executePlanSegTestCases = all testPlanSeg planSegTestCases
