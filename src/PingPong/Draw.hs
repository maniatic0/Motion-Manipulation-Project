module PingPong.Draw where

import Graphics.Gloss.Data.Color
import qualified Graphics.Gloss as G
import Codec.Picture( PixelRGBA8( .. ), writePng )

convertColor :: Color -> PixelRGBA8
convertColor c = 
  let (r, g, b, a) = rgbaOfColor c
      f x = fromInteger $ round $ 255 * x
  in  PixelRGBA8 (f r) (f g) (f b) (f a)

cRoom, cTable, cNet :: Color
cRoom  = G.makeColor 0.9 0.9 1.0 1
cTable = G.makeColor 0.2 0.4 0.1 1
cNet   = G.makeColor 0.2 0.2 0.2 1
cBall  = G.orange  