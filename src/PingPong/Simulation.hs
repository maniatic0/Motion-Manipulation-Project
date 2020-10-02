module PingPong.Simulation where

import PingPong.Model
import PingPong.Draw

import Data.Geometry hiding (init, head)
import Data.Geometry.Matrix
import Data.Geometry.Transformation

import Data.Ext
import Data.List hiding (intersect)
import Control.Lens hiding (snoc)

import Convert

import Data.Vinyl.CoRec





startBall :: BallState
startBall = BallState (Point2 0 1) (Vector2 0.88 0)

-- updating

update :: Float -> State -> IO State
update f st = do
  let op1 = p1 st
      op2 = p2 st
      ob  = ball st
  m1 <- action op1 ob (arm op1)
  m2 <- action op2 ob (arm op2)
  let np1 = op1 {arm = performMotion f m1 $ arm op1}
      np2 = op2 {arm = performMotion f m2 $ arm op2}
      sgs = net : listEdges table ++ listEdges room
      nb  = ballStep f sgs ob -- collect all relevant segments
  return $ st { p1   = np1
              , p2   = np2
              , ball = nb
              }

performMotion :: Float -> Motion -> Arm -> Arm 
performMotion f m a = performMotionRaw f (map cap m) a
  where
    cap :: Float -> Float
    cap x | x < -1    = -1
          | x >  1    =  1
          | otherwise =  x

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
ballStep :: Float -> [LineSegment 2 () Float] -> BallState -> BallState
ballStep f obs st = 
  let op  = loc st
      np  = loc st .+^ f *^ dir st
--      ts  = OpenLineSegment (op :+ ()) (np :+ ())
      ts  = LineSegment (Closed $ op :+ ()) (Open $ np :+ ())
--      ibs = filter (intersects ts) obs 
      ibs = filter (intersectsBla ts) obs 
      its = map (intersectionFraction ts) ibs
      tps = sortOn fst $ zip its ibs
  in case ibs of []   -> simpleBallStep f st
                 _    -> let ib = snd $ head tps
                             t = intersectionFraction ts ib
                             d = 0.001
                         in id
--                          $ ballStep ((1 - t) * f) obs
                          $ ballStep ((1 - t - d) * f) obs
                          $ simpleBallStep (d * f) 
                          $ reflect ib
                          $ simpleBallStep (t * f) st

-- ball step
simpleBallStep :: Float -> BallState -> BallState
simpleBallStep f st = st { loc = loc st .+^ f *^ dir st
                         , dir = decay *^ dir st ^+^ f *^ down
                         }
  where decay = 1 - 0.0005

reflect :: LineSegment 2 () Float -> BallState -> BallState
reflect s st = st {dir = reflectVector (dir st) (s ^. start ^. core .-. s ^. end ^. core)}

-- | Reflect vector 'a' in a line with direction vector 'b'.
reflectVector :: Vector 2 Float -> Vector 2 Float -> Vector 2 Float
reflectVector a b = reflection (angle (Vector2 1 0) b) `transformBy` a

angle :: Vector 2 Float -> Vector 2 Float -> Float
angle a b = acos $ signorm a `dot` signorm b

down :: Vector 2 Float
down = Vector2 0 (-1)

-- | Determine, for two intersecting line segments, at what fraction of the first segment
--   the intersection point lies.
intersectionFraction :: LineSegment 2 () Float -> LineSegment 2 () Float -> Float
intersectionFraction a b | intersectionFraction' a b < 0 = 0 -- error "< 0"
                         | intersectionFraction' a b > 1 = 1 -- error "< 1"
                         | otherwise = intersectionFraction' a b

intersectionFraction' :: LineSegment 2 () Float -> LineSegment 2 () Float -> Float
intersectionFraction' a b | (asA (intersect a b) :: Maybe (Point 2 Float)) == Nothing = 0
                          | otherwise = 
  let Point2 x1 y1        = a ^. start ^. core
      Just (Point2 x2 y2) = asA $ intersect a b
      Point2 x3 y3        = a ^. end ^. core
  in if x1 == x3 then (y2 - y1) / (y3 - y1)
                 else (x2 - x1) / (x3 - x1)

h, s :: LineSegment 2 () Float
h = OpenLineSegment (Point2 0 0 :+ ()) (Point2 10   0  :+ ())
s = OpenLineSegment (Point2 3 3 :+ ()) (Point2  7 (-2) :+ ())

test = listEdges table

-- test collisions ball and net

-- test collisions ball and table

-- test collisions players and table

-- test collisions ball and players
-- move simultaneously?

intersectsBla :: LineSegment 2 () Float -> LineSegment 2 () Float -> Bool
intersectsBla a b =
  let Point2 ax1' ay1' = a ^. start ^. core
      Point2 ax2' ay2' = a ^. end   ^. core
      Point2 bx1' by1' = b ^. start ^. core
      Point2 bx2' by2' = b ^. end   ^. core
      ax1 = min ax1' ax2'
      ax2 = max ax1' ax2'
      ay1 = min ay1' ay2'
      ay2 = max ay1' ay2'
      bx1 = min bx1' bx2'
      bx2 = max bx1' bx2'
      by1 = min by1' by2'
      by2 = max by1' by2'
  in (ax1 < bx1 && ax2 > bx1 && by1 < ay1 && by2 > ay1) 
  || (ax1 < bx2 && ax2 > bx2 && by1 < ay1 && by2 > ay1) 
  || (ax1 < bx1 && ax2 > bx1 && by1 < ay2 && by2 > ay2) 
  || (ax1 < bx2 && ax2 > bx2 && by1 < ay2 && by2 > ay2) 
  || (ay1 < by1 && ay2 > by1 && bx1 < ax1 && bx2 > ax1) 
  || (ay1 < by2 && ay2 > by2 && bx1 < ax1 && bx2 > ax1) 
  || (ay1 < by1 && ay2 > by1 && bx1 < ax2 && bx2 > ax2) 
  || (ay1 < by2 && ay2 > by2 && bx1 < ax2 && bx2 > ax2) 

