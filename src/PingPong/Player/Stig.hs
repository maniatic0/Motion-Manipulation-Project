-- By Christian Oliveros and Minmin Chen
module PingPong.Player.Stig (stig, test) where

import Control.Lens
import Data.Bool (bool)
import Data.Ext
import Data.Fixed (mod')
import Data.Geometry
import Data.Geometry.Vector.VectorFamilyPeano
import Data.Maybe
import Graphics.Gloss (Color, makeColor)
import PingPong.Model
import PingPong.Player

import Debug.Trace

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
tau :: Float
tau = 2 * pi

-- | Normalize Angle in Radians between (-pi, pi]
-- Based on https://stackoverflow.com/a/2323034
normalizeAngle :: Float -> Float
normalizeAngle a =
  let a1 = mod' a tau
      a2 = mod' (a + tau) tau
   in bool a2 (a2 - tau) (a2 > pi)

-- | Shortest Distance Between Two Angles in Radians [-pi, pi)
deltaAngle :: Float -> Float -> Float
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
    time :: r,
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
    r = d ^-^ n ^* (2 * dot d n)
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
    tc = time interception
    tcScaled = t0 + dt * tc -- To get it back into scale
    pc = trace ("Interception " ++ show interception ++ " Scaled Time " ++ show tcScaled) (point interception)
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
    (lcDirNormalized, lcNorm) = trace ("Col Line " ++ show p0c ++ " to " ++ show p1c ++ " at " ++ show a) $ normalizeVector lcDir
    -- Collision
    halfVectorUnNormalized = pv ^+^ lcv
    (halfVector, halfNorm) = normalizeVector halfVectorUnNormalized
    vd = pv ^-^ lcv -- Difference of velocities
    --rVd = reflect vd $ bool halfVector lcDirNormalized (lcNorm > 0) -- Use line as normal for reflection if possible. In case the line degenerated to a point use HalfVector
    rVd = reflect vd lcDirNormalized -- Use line as normal for reflection if possible. In case the line degenerated to a point use HalfVector
    cvd = rVd ^+^ lcv -- Collision Velocity Direction
    pCol = pc .+^ (cvd ^* (t1 - tcScaled))

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

-- End of Simulation Helpers

-- | Stig's player
stig :: Player
stig = Player stigArm stigFoot stigAction stigCollide

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
  (Num r, Floating r, Ord r, Eq r, Show r) =>
  (r, Point 2 r, LineSegment 2 () r) ->
  (r, Point 2 r, LineSegment 2 () r) ->
  Point 2 r
stigCollide = bool (error "Stig Collide Failed a Test Case") movingBallMovingLineCollide completeCheck
  where

    generateTestState :: (Num r, Floating r, Ord r) => r -> (r, r) -> (r, r) -> (r, r) -> (r, Point 2 r, LineSegment 2 () r)
    generateTestState t (px, py) (pl0x, pl0y) (pl1x, pl1y) = (t, Point2 px py, ClosedLineSegment (Point2 pl0x pl0y :+ ()) (Point2 pl1x pl1y :+ ()))
    
    testCases = [
      (Point2 0 0, generateTestState 0 (0, 0) (1, -1) (1, 1), generateTestState 1 (0, 0) (1, -1) (1, 1)),
      (Point2 0 0, generateTestState 0 (0, 0) (1, -1) (1, 1), generateTestState 1 (2, 0) (1, -1) (1, 1)),
      (Point2 (-1) 0, generateTestState 0 (0, 0) (1, -1) (1, 1), generateTestState 1 (1, 0) (0, -1) (0, 1)),
      (Point2 1 1, generateTestState 0 (0, 0) (0, -1) (2, 1), generateTestState 1 (2, 0) (0, -1) (2, 1)),
      (Point2 (-1) 1, generateTestState 0 (0, 0) (0, -1) (2, 1), generateTestState 1 (0, 0) (-2, -1) (0, 1))
      ]

    checkCollision :: (Num r, Floating r, Ord r, Show r) => (Point 2 r, (r, Point 2 r, LineSegment 2 () r), (r, Point 2 r, LineSegment 2 () r)) -> (Bool, Diff (Point 2) r, Point 2 r)
    checkCollision (ans, s1, s2) = (diffX == 0 && diffY == 0, diff, c)
      where
        c = movingBallMovingLineCollide s1 s2
        diff = c .-. ans
        diffX = globalThreshold 0 $ abs $ view xComponent diff
        diffY = globalThreshold 0 $ abs $ view yComponent diff

    performTest :: (Num r, Floating r, Ord r, Show r) => (Point 2 r, (r, Point 2 r, LineSegment 2 () r), (r, Point 2 r, LineSegment 2 () r)) -> Bool
    performTest testCase@(ans, s1, s2) = bool (error showError) True correct
      where
        (correct, diff, p) = checkCollision testCase
        showError 
          = "Expected " ++ show ans ++ " but got " 
            ++ show p ++ " with Diff " ++ show diff
            ++ " \n\tFor case:\n\t" ++ show s1 ++ "\n\t->\n\t" ++ show s2

    completeCheck = all performTest testCases    
  
test 
  = stigCollide (0, Point2 0.3 1.0, ClosedLineSegment (Point2 0.1 2.1 :+ ()) (Point2 (-0.5) 0.9 :+ ())) 
    (1, Point2 1.2 0.8, ClosedLineSegment (Point2 (-0.2) 2.2 :+ ()) (Point2 (-0.3) 1.1 :+ ()))

stigAction :: BallState -> Arm -> IO Motion
stigAction bs arm =
  return $
    let xdir = view xComponent $ dir bs
        toRest = armToStigRestMotion arm
        motion = bool [1, -1, 1, -1] toRest (xdir > 0)
     in applyMotionLimits motion -- Velocity limits