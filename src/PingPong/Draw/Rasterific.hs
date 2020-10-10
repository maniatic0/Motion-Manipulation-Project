module PingPong.Draw.Rasterific where

import PingPong.Model
import PingPong.Draw

import Codec.Picture( PixelRGBA8( .. ), writePng )
import qualified Graphics.Rasterific as R
import Graphics.Rasterific.Texture

import Control.Lens

import Data.Geometry
import Data.Geometry.Point
import Data.Geometry.Polygon
import Data.Geometry.LineSegment
import Data.Geometry.PlanarSubdivision

import Algorithms.Geometry.PolygonTriangulation.Triangulate

import Data.Ext
import Data.Foldable
import qualified Data.CircularSeq as C
import qualified Data.LSeq as L

import Graphics.Text.TrueType( Font, loadFontFile )


import           GHC.TypeNats

type Color = PixelRGBA8
type Picture = R.Drawing PixelRGBA8 ()


linkWidth, jointRadius, ballRadius :: Float
linkWidth = 2
jointRadius = 2
ballRadius = 4
wRoom = 1



drawState :: Font -> State -> Picture
drawState font m = do drawStatic
                      drawPlayer (p1 m) True
                      drawPlayer (p2 m) False
                      drawBall $ ball m
                      drawInfo font (name $ p1 m) (name $ p2 m)
--                 withColor (convertColor cRoom) $ R.fill $ R.rectangle (R.V2 100 100) 200 100

--center :: IsTransformable a => a -> a
center :: (IsTransformable g, Num (NumType g), Dimension g ~ 2) => g -> g
-- center = id
-- center = scaleUniformlyBy 200 . translateBy (Vector2 0 (-1))
center = scaleUniformlyBy 320 
       . scaleBy (Vector2 1 (-1))
       . translateBy (Vector2 3 (-3))


drawStatic :: Picture
drawStatic = do drawRoom
                drawTable
                drawNet

drawInfo :: Font -> String -> String -> Picture
drawInfo font n1 n2 = do 
  withColor (convertColor cTable) $ R.printTextAt font (R.PointSize 36) (R.V2 300 100) n2
  withColor (convertColor cTable) $ R.printTextAt font (R.PointSize 48) (R.V2 900 100) "VS"
  withColor (convertColor cTable) $ R.printTextAt font (R.PointSize 36) (R.V2 1300 100) n1
  return ()


drawRoom :: Picture
drawRoom = withColor (convertColor cRoom)
         $ R.fill
         $ rasterifyPolygon $ center room -- Line [(-1, 0), (1, 0), (1, 0.5), (-1, 0.5)]

drawTable :: Picture
drawTable = do withColor (convertColor cTable) $ R.fill $ rasterifyPolygon $ center table -- Line [(-1, 0), (1, 0), (1, 0.5), (-1, 0.5)]
--               withColor (convertColor cTable) $ R.stroke wRoom R.JoinRound (R.CapRound, R.CapRound)  $ rasterifyPolygon $ center table -- Line [(-1, 0), (1, 0), (1, 0.5), (-1, 0.5)]

drawNet :: Picture
drawNet = withColor (convertColor cNet)
        $ R.stroke wRoom R.JoinRound (R.CapRound, R.CapRound)
        $ rasterifySegment $ center net -- Line [(0, 0.5), (0, 0.6)]

drawBall :: BallState -> Picture
drawBall s = withColor (convertColor cBall) $ R.fill $ R.circle (rasterifyPoint $ center $ loc s) ballRadius

drawPlayer :: Player -> Bool -> Picture
drawPlayer p b = drawArm (arm p) (foot p) b


drawArm :: Arm -> Float -> Bool -> Picture
drawArm a d b = drawArm' a $ map (center . playerTransform d b) $ evaluate a
  where
    drawArm' :: Arm -> [Point 2 Float] -> Picture
    drawArm' (Link  l _ : rest) (p : q : qs) = sequence_ [drawLink p q $ convertColor l, drawArm' rest $ q : qs]
    drawArm' (Joint j _ : rest) (p :     ps) = sequence_ [drawArm' rest ps, drawJoint p $ convertColor j]
    drawArm' _ _ = return ()

playerTransform :: (IsTransformable g, NumType g ~ Float, Dimension g ~ 2) => Float -> Bool -> g -> g
--playerTransform :: Float -> Bool -> Point 2 Float -> Point 2 Float
--playerTransform :: (IsTransformable g, Num (NumType g), Dimension g ~ 2) => Float -> Bool -> g -> g
playerTransform d True = translateBy $ Vector2 d 0
playerTransform d False = scaleBy (Vector2 (-1) 1) . translateBy (Vector2 d 0)


drawLink :: Point 2 Float -> Point 2 Float -> Color -> Picture
drawLink p q c = withColor c
               $ R.stroke linkWidth R.JoinRound (R.CapRound, R.CapRound) 
               $ R.line (rasterify p) (rasterify q)

drawJoint :: Point 2 Float -> Color -> Picture
drawJoint p c = withColor c
              $ R.fill
              $ R.circle (rasterifyPoint p) jointRadius



withColor :: Color -> Picture -> Picture
withColor c = R.withTexture $ uniformTexture c

-- rasterifySegment






-- conversion to Rasterific geometry types

class Rasterifiable a b where
  rasterify :: a -> b

instance Real r => Rasterifiable (Point 2 r) (R.Point) where
  rasterify = rasterifyPoint

instance RealFrac r => Rasterifiable (Polygon t p r) [R.Primitive] where
  rasterify = rasterifyPolygon

instance RealFrac r => Rasterifiable (PolyLine 2 p r) [R.Primitive] where
  rasterify = rasterifyPolyLine

instance Real r => Rasterifiable (LineSegment 2 p r) [R.Primitive] where
  rasterify = rasterifySegment

instance (Rasterifiable a c, Rasterifiable b c) => Rasterifiable (Either a b) c where
  rasterify (Left  l) = rasterify l
  rasterify (Right r) = rasterify r









rasterifyPoint :: Real r => Point 2 r -> R.Point
rasterifyPoint (Point2 x y) = R.V2 (realToFrac x) (realToFrac y)

rasterifyPolygon :: (RealFrac) r => Polygon t p r -> [R.Primitive]
rasterifyPolygon p = 
  let t  = triangulate undefined p
      fs = toList $ faces' t
      is = filter (\f -> t ^. dataOf f == Inside) fs
      ts = map _core $ map (flip rawFaceBoundary t) is
  in concat $ map rasterifyConvexPolygon ts

rasterifyConvexPolygon :: RealFrac r => Polygon t p r -> [R.Primitive]
rasterifyConvexPolygon (SimplePolygon s) = R.polygon $ rasterifyCSeq s
rasterifyConvexPolygon (MultiPolygon s ps) = concat $ (R.polygon $ rasterifyCSeq s) : map rasterify ps

rasterifyPolyLine :: (RealFrac) r => PolyLine 2 p r -> [R.Primitive]
rasterifyPolyLine (PolyLine s) = R.polyline $ rasterifyLSeq s
  
rasterifyCSeq :: Real r => C.CSeq (Point 2 r :+ p) -> [R.Point]
rasterifyCSeq = map rasterifyPoint . map _core . toList

rasterifyLSeq :: Real r => L.LSeq 2 (Point 2 r :+ p) -> [R.Point]
rasterifyLSeq = map rasterifyPoint . map _core . toList

rasterifySegment :: Real r => LineSegment 2 p r -> [R.Primitive]
rasterifySegment s = R.line (rasterify $ s ^. start ^. core) (rasterify $ s ^. end ^. core)


