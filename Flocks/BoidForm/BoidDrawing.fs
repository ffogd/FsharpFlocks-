namespace BoidForm

open System.Drawing
open System.Drawing
open System.Drawing.Drawing2D

open Flocks.Boids

module BoidDrawing =

    type Drawing =
        abstract Draw : Graphics -> unit

    let drawing f =
      { new Drawing with 
          member x.Draw(gr) = f(gr) }
      
    let emptyDrawing =
      { new Drawing with 
          member x.Draw(gr) = () }

    let pen = new Pen(Color.Black)

    let inline drawBoid (brush1, brush2) epsilon boids =
        drawing(fun g ->   
          boids
          |>Seq.iter (fun boid ->
              let (x,y) = boid.position
              g.TranslateTransform(float32 x, float32 y)
              if boid.bounded then
                g.FillEllipse(brush1, 0, 0, 6, 6)
              else
                g.FillEllipse(brush2, 0, 0, 6, 6)
              g.DrawEllipse(pen, (-epsilon / 2) + 3 , (-epsilon / 2) + 3, epsilon, epsilon)
              g.TranslateTransform(-(float32 x), -(float32 y))))
