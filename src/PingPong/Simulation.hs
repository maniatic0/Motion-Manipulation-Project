module PingPong.Simulation where

import PingPong.Model
import PingPong.Draw

import Data.Geometry hiding (init, head, zero, replicate)
import Data.Geometry.Matrix
import Data.Geometry.Transformation

import Data.Fixed
import Data.Ext
import Data.List hiding (intersect)
import Data.Foldable
import Control.Lens hiding (snoc)

import Convert

import Data.Vinyl.CoRec

import Debug.Trace

import System.Random

startBall :: Bool -> IO BallState
startBall p = do
  h <- randomRIO (0.9, 1)
  x <- randomRIO (1.3, 1.4)
  y <- randomRIO (-0.1, 0)
  return $ BallState (Point2 0 h) (Vector2 (if p then x else -x) y)

-- | Maximum speed in radians per second of joint rotation.
maxSpeed :: Float
maxSpeed = 2

-- Duration of non-rally game phases in seconds
beforeGameTime  = 4
beforeRallyTime = 4
afterRallyTime  = 2
afterGameTime   = 10

-- updating

getPlayer :: Bool -> State -> Player
getPlayer True  = p1
getPlayer False = p2

act :: Bool -> State -> IO Motion
act b st = case phase st of BeforeGame _  -> stretch (getPlayer b st) (time st) $ arm $ getPlayer b st
                            BeforeRally _ -> straightenUp b st
                            DuringRally   -> actDuringRally b st
                            AfterGame _   -> dance (getPlayer b st) (time st) $ arm $ getPlayer b st
                            _             -> return $ replicate 5 0

straightenUp :: Bool -> State -> IO Motion
straightenUp True  st = return $ straighten (p1 st)
straightenUp False st = return $ straighten (p2 st)

straighten :: Player -> Motion
straighten p = 
  let vals = map modAngle $ map (\(Joint _ v) -> v) $ filter isJoint $ arm p 
      gals = map modAngle $ map (\(Joint _ v) -> v) $ filter isJoint $ initArm p 
  in zipWith (\v g -> modAngle $ g - v) vals gals

modAngle :: Float -> Float
modAngle x = (x + pi) `mod'` (2 * pi) - pi

actDuringRally :: Bool -> State -> IO Motion
actDuringRally True  st = action (p1 st) (time st) (head $ hits st) (ball st) (arm $ p1 st)
actDuringRally False st = act True (flipState st)
-- fmap flipMotion $
-- don't flip resulting motion -> motion is always in local perspective

update :: Float -> State -> IO State
update deltaTime st = do
  om1 <- act True  st
  om2 <- act False st
  let initialTime  = time st
      goalTime     = initialTime + deltaTime
      initialState = st {m1 = om1, m2 = om2}
      finalState   = case phase st of
                       BeforeRally _ -> updateUntilGhost goalTime initialState {frame = frame st + 1}
                       _ -> updateUntil goalTime initialState {frame = frame st + 1}
  perturbedState <- perturb deltaTime finalState
  updatePhase deltaTime perturbedState

-- update the phase: count down timer, and if reached 0, take approriate action
-- if phase is DuringRally, then check the last thing that was hit.
  -- need to know history???
updatePhase :: Float -> State -> IO State
updatePhase delta st = f $ phase st
  where 
    f (BeforeGame t)  | t > delta = return $ st {phase = BeforeGame  $ t - delta}
                      | otherwise = initBeforeRally st 
    f (BeforeRally t) | t > delta = return $ st {phase = BeforeRally $ t - delta}
                      | otherwise = initDuringRally st
    f (AfterRally t)  | t > delta = return $ st {phase = AfterRally  $ t - delta}
                      | otherwise = initBeforeRally st
    f (AfterGame t)   | t > delta = return $ st {phase = AfterGame   $ t - delta}
                      | otherwise = return $ st -- the game is over, simulation should stop
    f DuringRally     | testScore (map snd $ hits st) (view xCoord $ loc $ ball st) == Nothing = return $ st
                      | otherwise = updateScore (unJust $ testScore (map snd $ hits st) (view xCoord $ loc $ ball st)) (score st) st

initBeforeGame :: State -> IO State
initBeforeGame st = return $ st { phase = BeforeGame beforeGameTime
                                , score = (0, 0)
                                , ball  = BallState (Point2 (-1) 0.6) (Vector2 0.4 1)
                                , p1    = (p1 st) {initArm = arm (p1 st)}
                                , p2    = (p2 st) {initArm = arm (p2 st)}
                                }

initBeforeRally :: State -> IO State
initBeforeRally st = do
  b <- startBall True
  return $ st { phase = BeforeRally beforeRallyTime
              , ball  = b {dir = Vector2 0 0}
              }

initDuringRally :: State -> IO State
initDuringRally st = do
  let (i, j) = score st
      p      = (i + j) `mod` 4 < 2
  b <- startBall p
  return $ st { phase = DuringRally
              , hits  = [(0, Bat $ if p then Opponent else Self)] 
              , ball  = (ball st) {dir = dir b}
              }

initAfterRally :: State -> IO State
initAfterRally st = return $ st {phase = AfterRally afterRallyTime}

initAfterGame :: State -> IO State
initAfterGame st = return $ st {phase = AfterGame  afterGameTime}

unJust (Just x) = x

testScore :: [Item] -> Float -> Maybe Bool
testScore [] _ = Nothing
testScore [_] _ = Nothing
testScore (Table Self : Bat Opponent : _) _ = Nothing
testScore (Table Opponent : Bat Self : _) _ = Nothing
testScore (Bat Self : Table Self : _) _ = Nothing
testScore (Bat Opponent : Table Opponent : _) _ = Nothing
testScore (_ : Bat Opponent : _) x | x > 0 && x < 1 = Just False
                                   | otherwise      = Just True
testScore (_ : Bat Self : _) x | x > -1 && x < 0 = Just True
                               | otherwise       = Just False
testScore (_ : Table Opponent : _) _ = Just True
testScore (_ : Table Self : _) _ = Just False
testScore _ _ = Nothing

updateScore :: Bool -> (Int, Int) -> State -> IO State
updateScore True  (a, b) st | a >= 10 && b <  a  = initAfterGame $ st {score = (a + 1, b)}
updateScore False (a, b) st | a <  b  && b >= 10 = initAfterGame $ st {score = (a, b + 1)}
updateScore True  (a, b) st = initAfterRally $ st {score = (a + 1, b)}
updateScore False (a, b) st = initAfterRally $ st {score = (a, b + 1)}

perturb :: Float -> State -> IO State
perturb deltaTime st = do
  dx <- randomRIO (-amount, amount)
  dy <- randomRIO (-amount, amount)
  return $ st {ball = (ball st) {dir = dir (ball st) ^+^ Vector2 dx dy}}
    where amount = 0.025 * deltaTime

-- | Update state using fixed motion until the goal time.
--   Will recurse until the next collision event is later than the goal time.
updateUntil :: Float -> State -> State
updateUntil deadline st0 | deadline == time st0 = st0
                         | otherwise =
  let st1 = updateUntilRaw deadline st0
      t0  = time st0
      t1  = time st1
      b0  = loc $ ball st0
      b1  = loc $ ball st1
      collide (i, s0, s1) = collide' i (t0, b0, s0) (t1, b1, s1)
      repeated (t, i, _) = i /= Air && (fst $ head $ hits st0) >= t0 && i == (snd $ head $ hits st0)
      candidates = sort $ filter (not . repeated) $ map collide $ zip3 items (segmentsAt st0) (segmentsAt st1)
      (t, i, v) = head $ candidates ++ [(t1, Air, dir $ ball st1)]
  in -- traceShow (t, v) $  
     updateUntil deadline $ updateUntilRaw t st0 { ball = (ball st0) {dir = v}
                                                 , hits = newHits (hits st0) (t, i)
                                                 }

items :: [Item]
items = map item $ [0..]

item :: Int -> Item
item 0 = Bat Self
item 1 = Bat Opponent
item 2 = Table Self
item 3 = Table Opponent
item x = Other x

newHits :: [(Float, Item)] -> (Float, Item) -> [(Float, Item)]
newHits os (_, Air) = os
newHits os n        = n : os

-- | Updates state without checking for collisions.
updateUntilRaw :: Float -> State -> State
updateUntilRaw deadline st | deadline == time st = st
                           | otherwise =
  let f   = deadline - time st
      op1 = p1 st
      op2 = p2 st
      ob  = ball st
      np1 = op1 {arm = performMotion f (m1 st) op1}
      np2 = op2 {arm = performMotion f (m2 st) op2}
      nb  = simpleBallStep f ob
  in st { time = deadline
        , p1   = np1
        , p2   = np2
        , ball = nb
        }

-- | Updates state without checking for collisions.
updateUntilGhost :: Float -> State -> State
updateUntilGhost deadline st | deadline == time st = st
                           | otherwise =
  let f   = deadline - time st
      op1 = p1 st
      op2 = p2 st
      ob  = ball st
      np1 = op1 {arm = performMotionRaw f (m1 st) (arm op1)}
      np2 = op2 {arm = performMotionRaw f (m2 st) (arm op2)}
      nb  = ob
  in st { time = deadline
        , p1   = np1
        , p2   = np2
        , ball = nb
        }


segmentsAt :: State -> [LineSegment 2 () Float]
segmentsAt st =  (last . toList . edgeSegments) (playerGeom True  $ p1 st)
              :  (last . toList . edgeSegments) (playerGeom False $ p2 st)
              :  listEdges table 
              ++ listEdges room
              ++ [net] 
              ++ (init . toList . edgeSegments) (playerGeom True  $ p1 st)
              ++ (init . toList . edgeSegments) (playerGeom False $ p2 st)
 

playerGeom :: Bool -> Player -> PolyLine 2 () Float
playerGeom b p = playerTransform (foot p) b $ evaluateP (arm p) 


playerTransform :: (IsTransformable g, NumType g ~ Float, Dimension g ~ 2) => Float -> Bool -> g -> g
playerTransform d True = translateBy $ Vector2 d 0
playerTransform d False = scaleBy (Vector2 (-1) 1) . translateBy (Vector2 d 0)

performMotion :: Float -> Motion -> Player -> Arm 
performMotion f m p | all (== 0) m = arm p
                    | otherwise =
  let na = performMotionRaw f (map cap m) $ arm p
  in if or [ intersectsExact s t
           | s <- filter (not . degenerate) $ armSegments p {arm = na} True
           , t <- listEdges table ++ listEdges room
           ]
     then performMotion f (strip0 m) p
     else na

strip0 :: [Float] -> [Float]
strip0 [] = []
strip0 (x : xs) | x /= 0 = 0 : xs
                | x == 0 = x : strip0 xs

cap :: Float -> Float
cap x | x < -maxSpeed = -maxSpeed
      | x >  maxSpeed =  maxSpeed
      | otherwise     =  x


intersectsExact :: LineSegment 2 () Float -> LineSegment 2 () Float -> Bool
--intersectsExact s t | converf s `intersects` converf t = traceShow (s, t) $ True
--                    | otherwise                        = False
intersectsExact s t = converf s `intersects` converf t

converf :: LineSegment 2 () Float -> LineSegment 2 () Rational
converf = endPoints . core . traverse %~ realToFrac


degenerate :: Eq r => LineSegment 2 () r -> Bool
degenerate s = s ^. start . core == s ^. end . core


{-  
  let rs, rt :: LineSegment 2 () Rational
      rs = s & endPoints %~ (traverse %~ realToFrac)
      rt = t & endPoints %~ (traverse %~ realToFrac)
  in intersects rs rt
-}

-- needs to check:
-- * for too fast motion
-- * collision with table
-- * self-collision
-- * reaching over origin?

performMotionRaw :: Float -> Motion -> Arm -> Arm 
performMotionRaw f m (l@(Link _ _) : arm) = l : performMotionRaw f m arm
performMotionRaw f (x : xs) (Joint c a : arm) = Joint c (applyMotion f x a) : performMotionRaw f xs arm
performMotionRaw f _ arm = arm

applyMotion :: Float -> Float -> Float -> Float
applyMotion f x a = a + f * x



-- ball step
simpleBallStep :: Float -> BallState -> BallState
simpleBallStep f st = st { loc = loc st .+^ f *^ dir st
                         , dir = decay *^ dir st ^+^ 2 * f *^ down
                         }
  where decay = (1 - 0.05) ** f


-- | Reflect vector 'a' in a line with direction vector 'b'.
reflectVector :: Vector 2 Float -> Vector 2 Float -> Vector 2 Float
reflectVector a b = reflection (angle (Vector2 1 0) b) `transformBy` a

-- | Find the angle between two vectors, in counter-clockwise order, from the first to the second.
angle :: Vector 2 Float -> Vector 2 Float -> Float
angle (Vector2 x1 y1) (Vector2 x2 y2) = atan2 (x1 * y2 - y1 * x2) (x1 * x2 + y1 * y2)

down :: Vector 2 Float
down = Vector2 0 (-1)





collide' :: Item
         -> (Float, Point 2 Float, LineSegment 2 () Float) 
         -> (Float, Point 2 Float, LineSegment 2 () Float) 
         -> (Float, Item, Vector 2 Float)

collide' i (t0, b0, s0) (t1, b1, s1) = 
  case collisionPoints (t0, b0, s0) (t1, b1, s1)
  of []            -> (t1, Air, (b1 .-. b0) ^/ (t1 - t0))
     (p, s, t) : _ -> (t, i, collisionVelocity (t0, b0, s0) (t1, b1, s1) (p, s, t))

-- | For a given collision point and time, compute the velocity of the ball
--   at that point and time.
collisionVelocity :: (Float, Point 2 Float, LineSegment 2 () Float) 
                  -> (Float, Point 2 Float, LineSegment 2 () Float) 
                  -> (Point 2 Float, Float, Float)
                  -> Vector 2 Float
collisionVelocity (t0, b0, s0) (t1, b1, s1) (p, s, t') = 
  let t = (t' - t0) / (t1 - t0)
      c0 = s0 ^. start ^. core
      d0 = s0 ^. end   ^. core
      c1 = s1 ^. start ^. core
      d1 = s1 ^. end   ^. core
      vl =   ((1-t) *^ (d0 .-. origin) ^+^ t *^ (d1 .-. origin)) 
         ^-^ ((1-t) *^ (c0 .-. origin) ^+^ t *^ (c1 .-. origin))
      vs =   (((1-s) *^ (c1 .-. origin) ^+^ s *^ (d1 .-. origin)) 
         ^-^ ((1-s) *^ (c0 .-. origin) ^+^ s *^ (d0 .-. origin)))
         ^/  (t1 - t0)
      vb = (b1 .-. b0) ^/ (t1 - t0)
      vr = vb ^-^ vs
      vm = reflectVector vr vl
  in vm ^+^ vs

-- | Collect all points where collisions happen, and also report the fraction
--   of the segment and the time of each collision.
--   May return 0, 1, or 2 points.
collisionPoints :: (Float, Point 2 Float, LineSegment 2 () Float) 
                -> (Float, Point 2 Float, LineSegment 2 () Float) 
                -> [(Point 2 Float, Float, Float)]
collisionPoints (t0, b0, s0) (t1, b1, s1) = 
  let c0 = s0 ^. start ^. core
      d0 = s0 ^. end   ^. core
      c1 = s1 ^. start ^. core
      d1 = s1 ^. end   ^. core
      Point2 xb0 yb0 = b0
      Point2 xb1 yb1 = b1
      Point2 xc0 yc0 = c0
      Point2 xc1 yc1 = c1
      Point2 xd0 yd0 = d0
      Point2 xd1 yd1 = d1
      xa = xd0 - xc0
      ya = yd0 - yc0
      xb = xb0 - xc0 + xc1 - xb1
      yb = yb0 - yc0 + yc1 - yb1 
      xc = xc0 - xd0 + xd1 - xc1
      yc = yc0 - yd0 + yd1 - yc1
      xd = xb0 - xc0
      yd = yb0 - yc0
      i = xd * ya - yd * xa
      j = xd * yc - xb * ya - yd * xc + yb * xa
      k = yb * xc - xb * yc
      ts = solveQuadraticEquation k j i
      s t | zero $ xa + xc * t = (yd - yb * t) / (ya + yc * t)
          | zero $ ya + yc * t = (xd - xb * t) / (xa + xc * t)
          | otherwise = let s1 = (xd - xb * t) / (xa + xc * t)
                            s2 = (yd - yb * t) / (ya + yc * t)
                        in if 0 <= s1 && s1 <= 1 then s1 else s2 -- checkAlmostEqual s1 s2 
      ss = map s ts
      p (s, t) = origin .+^ (1-t) * (1-s) *^ (c0 .-. origin) 
                        .+^ (1-t) * s     *^ (d0 .-. origin) 
                        .+^ t     * (1-s) *^ (c1 .-. origin)
                        .+^ t     * s     *^ (d1 .-. origin)
      ps = map p $ zip ss ts
      ts' = map (\t -> (1 - t) * t0 + t * t1) ts
      psts = zip3 ps ss ts'
  in filter (\(p, s, t) -> 0 < s && s <= 1 && t0 < t && t <= t1) psts


checkAlmostEqual :: (Ord r, Floating r, Show r) => r -> r -> r
checkAlmostEqual a b | a == 0 && abs b > treshold = error message
                     | b == 0 && abs a > treshold = error message
                     | a == 0 || b == 0           = 0
                     | a / b > 1 + treshold       = error message
                     | b / a > 1 + treshold       = error message
                     | otherwise                  = a -- trace ("checking " ++ show a ++ " " ++ show b) a
  where treshold = 100
        message  = error $ "checkAlmostEqual: " ++ show a ++ " /= " ++ show b


-- | Solve equation of the form ax^2 + bx + c = 0.
--   Attempt at a somewhat robust implementation.
solveQuadraticEquation :: (Ord r, Enum r, Floating r, Show r) => r -> r -> r -> [r]
solveQuadraticEquation 0 0 0 = [0] -- [0..]
solveQuadraticEquation a 0 0 = [0]
solveQuadraticEquation 0 b 0 = [0]
solveQuadraticEquation 0 0 c = []
solveQuadraticEquation a b 0 = sort [0, -b / a]
solveQuadraticEquation a 0 c | (-c / a) <  0 = []
                             | (-c / a) == 0 = [0]
                             | (-c / a) >  0 = [sqrt (-c / a)]
solveQuadraticEquation 0 b c = [-c / b]
solveQuadraticEquation a b c | zero a || zero (a / b) || zero (a / c) = solveQuadraticEquation 0 b c
solveQuadraticEquation a b c = 
  let d = b^2 - 4 * a * c
      result | d == 0 = [-b / (2 * a)]
             | d >  0 = [(-b - sqrt d) / (2 * a), (-b + sqrt d) / (2 * a)]
             | otherwise = []
  in result
  -- trace ("soving equation " ++ show a ++ "x^2 + " ++ show b ++ "x + " ++ show c ++ " = 0") $ result  


-- | Test whether a floating point number is zero, taking rounding errors into account.
zero :: (Floating r, Ord r) => r -> Bool
zero x = abs x < epsilon

-- | Treshold for rounding errors in zero tests
--   TODO: Should be different depending on the type.
epsilon :: Floating r => r
epsilon = 0.0001
