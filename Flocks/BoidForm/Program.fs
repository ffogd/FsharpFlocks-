namespace BoidForm

open System
open System.Drawing
open System.Windows.Forms
open Microsoft.FSharp.Collections

open Flocks.KDTree
open Flocks.Boids
open BoidDrawing

module Main =
    let synchronize f = 
        let ctx = System.Threading.SynchronizationContext.Current 
        f (fun g arg ->
            let nctx = System.Threading.SynchronizationContext.Current 
            if ctx <> null && ctx <> nctx then ctx.Post((fun _ -> g(arg)), null)
            else g(arg) )

    type Microsoft.FSharp.Control.Async with 
      static member AwaitObservable(ev1:IObservable<'a>) =
        synchronize (fun f ->
          Async.FromContinuations((fun (cont,econt,ccont) -> 
            let rec callback = (fun value ->
              remover.Dispose()
              f cont value )
            and remover : IDisposable  = ev1.Subscribe(callback) 
            () )))
  
      static member AwaitObservable(ev1:IObservable<'a>, ev2:IObservable<'b>) = 
        synchronize (fun f ->
          Async.FromContinuations((fun (cont,econt,ccont) -> 
            let rec callback1 = (fun value ->
              remover1.Dispose()
              remover2.Dispose()
              f cont (Choice1Of2(value)) )
            and callback2 = (fun value ->
              remover1.Dispose()
              remover2.Dispose()
              f cont (Choice2Of2(value)) )
            and remover1 : IDisposable  = ev1.Subscribe(callback1) 
            and remover2 : IDisposable  = ev2.Subscribe(callback2) 
            () )))

    type InputParams = | Cohesion of float | Alignment of float | Scale of float | Separation of float | Velo of float | Epsilon of float
    let inline createParams input p =
        match input with
        | Cohesion v when v <> 0.0 ->     {p with cParam = v} 
        | Alignment v when v <> 0.0->    {p with aParam = v} 
        | Scale v when v <> 0.0->        {p with sScale = v} 
        | Separation v->    {p with sParam = v} 
        | Velo v->          {p with velo = v}
        | Epsilon v -> {p with epsilon = v}
        | _ -> p

    let iter  = iterationkd (drawBoid (Brushes.Blue, Brushes.Red)) 

    let test =
        let boundMin,boundMax = 0.0, 600.0

        let af = new BoidForm(ClientSize = Size((int boundMax), (int boundMax)+50), Visible = true)
        let cohesionLabel = new Label(Text ="cohesion",Left = 8, Top = 610, Width = 60, Height = 15 )
        let cohesionTextBox = new TextBox(Text = parms.cParam.ToString(), Left = 8, Top = 630, Width = 60)
        let alignmentLabel = new Label(Text ="alignment",Left = 70, Top = 610, Width = 60, Height = 15 )
        let alignmentTextBox = new TextBox(Text = parms.aParam.ToString(), Left = 70, Top = 630, Width = 60)

        let scaleLabel = new Label(Text ="separation scale",Left = 135, Top = 610, Width = 90, Height = 15 )
        let scaleTextBox = new TextBox(Text = parms.sScale.ToString(), Left = 135, Top = 630, Width = 60)
        
        let separationLabel = new Label(Text ="separation",Left = 230, Top = 610, Width = 60, Height = 15 )
        let separationTextBox = new TextBox(Text = parms.sParam.ToString(), Left = 230, Top = 630, Width = 60)
        
        let veloLabel = new Label(Text ="velocity",Left = 295, Top = 610, Width = 60, Height = 15 )
        let veloTextBox = new TextBox(Text = parms.velo.ToString(), Left = 295, Top = 630, Width = 60)

        let epsilonLabel = new Label(Text ="epsilon",Left = 360, Top = 610, Width = 60, Height = 15 )
        let epsilonTextBox = new TextBox(Text = parms.epsilon.ToString(), Left = 360, Top = 630, Width = 60)

        let evtClicks = 
            let parse f text = 
                let (ok, v) = System.Double.TryParse(text)
                if ok then Some(createParams (f v)) else None
            Event.merge (Event.map (fun _ -> parse Cohesion cohesionTextBox.Text) cohesionTextBox.TextChanged) (Event.map (fun _-> parse Alignment alignmentTextBox.Text) alignmentTextBox.TextChanged)
            |> Event.merge (Event.map (fun _ -> parse Scale scaleTextBox.Text) scaleTextBox.TextChanged)
            |> Event.merge (Event.map (fun _ -> parse Separation separationTextBox.Text) separationTextBox.TextChanged)
            |> Event.merge (Event.map (fun _ -> parse Velo veloTextBox.Text) veloTextBox.TextChanged)
            |> Event.merge (Event.map (fun _ -> parse Epsilon epsilonTextBox.Text) epsilonTextBox.TextChanged)
            |> Event.choose id
        af.Controls.AddRange([| (cohesionTextBox:>Control); (alignmentTextBox:>Control);(scaleTextBox:>Control); (separationTextBox:>Control); (veloTextBox:>Control); (epsilonTextBox:>Control);
                                (cohesionLabel:>Control); (alignmentLabel:>Control);(scaleLabel:>Control); (separationLabel:>Control); (veloLabel:>Control); (epsilonLabel:>Control)|])
        let swarmInit = 
            List.map (fun i ->makeboid i ) [0..300] |> fromListWithDepth

        //Start swarm after 10 steps.
        let swarmStart  = 
            List.fold (fun (facc,_) _-> 
                    (iter parms facc)) (iter parms swarmInit) [0..10]

        let rec waiting swarm p = async {
            let! evnt = Async.AwaitObservable (af.Paint, evtClicks)
            match evnt with
            | Choice1Of2(evntArg1) -> 
                let newSwarm, drawing = swarm 
                af.guiRefresh evntArg1.Graphics drawing
                do! waiting (iter p newSwarm) p
            | Choice2Of2(f) ->
                do! waiting swarm (f p)
                }
        (waiting swarmStart parms) |> Async.StartImmediate
        af

    [<STAThread>]
    do
        Application.EnableVisualStyles()
        Application.Run(test)
