namespace BoidForm

open System
open System.Drawing
open System.Windows.Forms
open Flocks.KDTree
open BoidForm.BoidDrawing
open System.Drawing.Drawing2D

type public BoidForm() as form =
    inherit Form()
  
    do
        form.SuspendLayout();
         
        form.SetStyle(ControlStyles.AllPaintingInWmPaint ||| ControlStyles.OptimizedDoubleBuffer, true)
        form.FormBorderStyle <- FormBorderStyle.FixedToolWindow
        form.StartPosition <- FormStartPosition.CenterScreen;
    
        let tmr = new Timers.Timer(Interval = 40.0)
        tmr.Elapsed.Add(fun _ -> form.Invalidate() )
        tmr.Start()

        form.Text <- "F# Flock"

        // render the form
        form.ResumeLayout(false)
        form.PerformLayout()

    member x.guiRefresh (e:Graphics)  (swarm : Drawing) = 
        e.FillRectangle(Brushes.White, Rectangle(Point(0,0), Size(x.ClientSize.Width, x.ClientSize.Height-40)))
        e.SmoothingMode <- SmoothingMode.AntiAlias
        swarm.Draw(e)

    