module PingPong.Player.ModelPlayer (modelPlayer) where

import PingPong.Model
import PingPong.Player

import Data.Geometry hiding (zero)
import Data.Ext
import Data.List
import Control.Lens
import Graphics.Gloss (Color, makeColor)

modelPlayer :: Player
modelPlayer = defaultPlayer
  { name = "Roland Jacobi"
  , collide = modelCollide
  }



modelCollide :: (Float, Point 2 Float, LineSegment 2 () Float) 
             -> (Float, Point 2 Float, LineSegment 2 () Float) 
             -> IO (Point 2 Float)
modelCollide (t0, b0, s0) (t1, b1, s1) = return $ 
  case collisionPoints (t0, b0, s0) (t1, b1, s1)
  of []            -> b1
     (p, s, t) : _ -> p .+^ (t1 - t) *^ collisionVelocity (t0, b0, s0) (t1, b1, s1) (p, s, t)


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
                        in if 0 <= s1 && s1 <= 1 then s1 else s2
      ss = map s ts
      p (s, t) = origin .+^ (1-t) * (1-s) *^ (c0 .-. origin) 
                        .+^ (1-t) * s     *^ (d0 .-. origin) 
                        .+^ t     * (1-s) *^ (c1 .-. origin)
                        .+^ t     * s     *^ (d1 .-. origin)
      ps = map p $ zip ss ts
      ts' = map (\t -> (1 - t) * t0 + t * t1) ts
      psts = zip3 ps ss ts'
  in filter (\(p, s, t) -> 0 < s && s <= 1 && t0 < t && t <= t1) psts

-- | Reflect vector 'a' in a line with direction vector 'b'.
reflectVector :: Vector 2 Float -> Vector 2 Float -> Vector 2 Float
reflectVector a b = reflection (angle (Vector2 1 0) b) `transformBy` a

-- | Find the angle between two vectors, in counter-clockwise order, from the first to the second.
angle :: Vector 2 Float -> Vector 2 Float -> Float
angle (Vector2 x1 y1) (Vector2 x2 y2) = atan2 (x1 * y2 - y1 * x2) (x1 * x2 + y1 * y2)


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

-- | Test whether a floating point number is zero, taking rounding errors into account.
zero :: (Floating r, Ord r) => r -> Bool
zero x = abs x < epsilon

-- | Treshold for rounding errors in zero tests
epsilon :: Floating r => r
epsilon = 0.0001