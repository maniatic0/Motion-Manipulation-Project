{-# LANGUAGE ScopedTypeVariables #-}

-- By Christian Oliveros and Minmin Chen
module PingPong.Player.Stig.GeometryHelpers where

import Control.Lens (view, (&), (.~), (^.))
import Data.Bool (bool)
import Data.Ext
import Data.Fixed (mod')
import Data.Geometry hiding (Above, Below, head)
import Data.Maybe
import Debug.Trace
import GHC.Float

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

-- | Convert a float point to a double point
pointFloat2Double :: Point 2 Float -> Point 2 Double
pointFloat2Double p = Point2 (float2Double $ view xCoord p) (float2Double $ view yCoord p)

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

-- | Angle from v1 to v2
-- https://scicomp.stackexchange.com/questions/27689/numerically-stable-way-of-computing-angles-between-vectors
angleVectorPrecise :: (Num r, RealFloat r) => Vector 2 r -> Vector 2 r -> r
angleVectorPrecise v1 v2 = atan2 (outer2D v1N v2N) (dot v1N v2N)
  where
    (v1N, _) = normalizeVector v1
    (v2N, _) = normalizeVector v2

-- | Angle from v1 to v2
-- https://scicomp.stackexchange.com/questions/27689/numerically-stable-way-of-computing-angles-between-vectors
angleVectorPreciseSigned :: (Num r, RealFloat r) => Vector 2 r -> Vector 2 r -> r
angleVectorPreciseSigned v1 v2 = sign * atan2 perp (dot v1N v2N)
  where
    (v1N, _) = normalizeVector v1
    (v2N, _) = normalizeVector v2
    perp = outer2D v1N v2N
    sign = bool 1 (-1) (perp < 0)

-- | Angle from v1 to v2
-- https://scicomp.stackexchange.com/questions/27689/numerically-stable-way-of-computing-angles-between-vectors
angleVectorPrecise2 :: (Num r, RealFloat r) => Vector 2 r -> Vector 2 r -> r
angleVectorPrecise2 v1 v2 = 2 * atan2 (norm $ v1N ^-^ v2N) (norm $ v1N ^+^ v2N)
  where
    (v1N, _) = normalizeVector v1
    (v2N, _) = normalizeVector v2

-- | Angle from v1 to v2
-- https://scicomp.stackexchange.com/questions/27689/numerically-stable-way-of-computing-angles-between-vectors
angleVectorPrecise2Signed :: (Num r, RealFloat r) => Vector 2 r -> Vector 2 r -> r
angleVectorPrecise2Signed v1 v2 = sign * 2 * atan2 (norm $ v1N ^-^ v2N) (norm $ v1N ^+^ v2N)
  where
    (v1N, _) = normalizeVector v1
    (v2N, _) = normalizeVector v2
    perp = outer2D v1N v2N
    sign = bool 1 (-1) (perp < 0)

-- | Angle from v1 to v2
-- https://scicomp.stackexchange.com/questions/27689/numerically-stable-way-of-computing-angles-between-vectors
angleVectorPrecise3 :: forall r. (Num r, RealFloat r) => Vector 2 r -> Vector 2 r -> r
angleVectorPrecise3 v1 v2 = 2 * atan (sqrt (numerator / denominator))
  where
    v1Norm = norm v1
    v2Norm = norm v2

    a = bool v2Norm v1Norm (v1Norm > v2Norm)
    b = bool v1Norm v2Norm (v1Norm > v2Norm)
    c = bool (norm (v2 ^-^ v1)) (norm (v1 ^-^ v2)) (v1Norm > v2Norm)

    nu = calcNu a b c

    numerator = ((a - b) + c) * nu
    denominator = (a + (b + c)) * ((a - c) + b)

    calcNu :: r -> r -> r -> r
    calcNu a0 b0 c0
      | b0 >= c0 && c0 >= 0 = c0 - (a0 - b0)
      | c0 > b0 && b0 >= 0 = b0 - (a0 - c0)
      | otherwise = error "Invalid triangle"

-- | Angle from v1 to v2
-- https://scicomp.stackexchange.com/questions/27689/numerically-stable-way-of-computing-angles-between-vectors
angleVectorPrecise3Signed :: forall r. (Num r, RealFloat r) => Vector 2 r -> Vector 2 r -> r
angleVectorPrecise3Signed v1 v2 = sign * 2 * atan (sqrt (numerator / denominator))
  where
    v1Norm = norm v1
    v2Norm = norm v2

    a = bool v2Norm v1Norm (v1Norm > v2Norm)
    b = bool v1Norm v2Norm (v1Norm > v2Norm)
    c = bool (norm (v2 ^-^ v1)) (norm (v1 ^-^ v2)) (v1Norm > v2Norm)

    nu = calcNu a b c

    numerator = ((a - b) + c) * nu
    denominator = (a + (b + c)) * ((a - c) + b)

    perp = outer2D v1 v2
    sign = bool 1 (-1) (perp < 0)

    calcNu :: r -> r -> r -> r
    calcNu a0 b0 c0
      | b0 >= c0 && c0 >= 0 = c0 - (a0 - b0)
      | c0 > b0 && b0 >= 0 = b0 - (a0 - c0)
      | otherwise = error "Invalid triangle"

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

-- | Normalize a vector and get its norm
normalizeVectorThreshold :: (Num r, Floating r, Ord r, Show r) => (r -> r -> r) -> Vector 2 r -> (Vector 2 r, r)
normalizeVectorThreshold t v = (n, len)
  where
    len = t 0 $ norm v
    n = bool (traceShow v $ Vector2 0 0) (v ^* (1 / len)) (len > 0)

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

-- | Rotate a 2D Vector by angle q
rotateVector2D :: (Num r, Floating r, Ord r, Show r) => r -> Vector 2 r -> Vector 2 r
rotateVector2D q v0 = Vector2 (x * c - y * s) (x * s + c * y)
  where
    c = cos q
    s = sin q
    x = view xComponent v0
    y = view yComponent v0

-- | Distance to a Range
data DistanceToRange r
  = Above r
  | Below r
  | Inside r
  deriving (Show)

-- | Gets the content of a DistanceToRange
getDistanceToRangeContent :: DistanceToRange r -> r
getDistanceToRangeContent (Above a) = a
getDistanceToRangeContent (Below a) = a
getDistanceToRangeContent (Inside a) = a

-- | Sgined Distance to Range [a,b] (a<=b)
signedDistanceToRange :: (Num r, Floating r, Ord r, Show r) => r -> r -> r -> DistanceToRange r
signedDistanceToRange a b t
  | above = Above $ b - t
  | below = Below $ a - t
  | otherwise = Inside $ min (t - a) (b - t)
  where
    above = b < t
    below = t < a

-- | Get a segments direction
segmentDir :: (Num r, Floating r, Ord r, Show r) => LineSegment 2 () r -> Vector 2 r
segmentDir s = n
  where
    (n, _) = normalizeVector $ (s ^. end . core) .-. (s ^. start . core)

-- | Invert a segment
segmentInvert :: (Num r, Floating r, Ord r, Show r) => LineSegment 2 () r -> LineSegment 2 () r 
segmentInvert l = ClosedLineSegment (l ^. end . core :+ ()) (l ^. start . core :+ ())

-- End of Geometry Helpers