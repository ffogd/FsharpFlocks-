//from Haskell Version https://github.com/mjsottile/publicstuff/tree/master/boids
namespace Flocks
open System

open Microsoft.FSharp.Math
open Microsoft.FSharp.Collections

module KDTree =

    type KDTreeNode<'a> =
        | Empty
        | Node of KDTreeNode<'a> * (float * float) * 'a * KDTreeNode<'a>
        //        left tree         position         data   Right tree
    
    let inline flatten tree =
        let s = System.Collections.Generic.Stack[tree]
        
        let rec loop (stack : Collections.Generic.Stack<KDTreeNode<'a>>) acc =
            match stack.Count>0 with
            | false -> acc
            | true ->
                match stack.Pop() with
                | Empty -> loop stack acc
                | Node(left, _, a, right) ->
                    stack.Push left
                    stack.Push right
                    loop stack (a :: acc)
        loop s


    let newKDTree = Empty

    let inline vecLessThan (a, b) (x, y)  = a < x && b < y
    let inline vecGreaterThan (a, b) (x, y)  = a > x && b > y

    let inline vecDimSelect (x, y) n =
        match n with
        | 0 -> x
        | 1 -> y
        | other -> failwith "invalid argument"  

    let inline kdtInBounds p bMin bMax =  (vecLessThan p bMax) && (vecGreaterThan p bMin)

    let inline kdtRangeSearch t bMin bMax =
        let rec inner t (current, next) acc =
            let nextfuncs = (next, current)
            match t with 
            | Empty -> acc
            | Node (left, npos, ndata, right) -> 
                match current npos < current bMin, 
                        current npos > current bMax with
                | true, _       -> inner right nextfuncs acc
                | _, true       -> inner left nextfuncs acc
                | false, false  ->
                    match kdtInBounds npos bMin bMax with
                    | true      -> inner right nextfuncs (inner left nextfuncs ((npos, ndata) :: acc))
                    | false     -> inner right nextfuncs (inner left nextfuncs acc)
        inner t (fst, snd) []

module Boids =
    open KDTree

    type Boid = { identifier : int; position : float * float; velocity : float * float; bounded : bool}

    // create KD Tree from boids list.
    let inline fromListWithDepth l =
        let rec loop boidPoints d =
            match Array.isEmpty boidPoints with
            | true -> Empty
            | false  ->
                let axis = d % 2 
                Array.sortInPlaceBy (fun boid -> vecDimSelect boid.position axis) boidPoints
                let index = Array.length boidPoints / 2
                if index = 0 then
                    let dataBoid =  boidPoints.[0]
                    Node (Empty, dataBoid.position, dataBoid, Empty)
                else
                    let lf, r =     boidPoints.[0..index - 1], boidPoints.[index..]
                    let dataBoid =  r.[0]
                    Node(loop lf  (d + 1) , dataBoid.position, dataBoid, loop r.[1..] (d + 1))
        loop (l |> Array.ofSeq) 0 

    type Params = {velo : float; cParam : float;  sParam : float;
                    sScale : float; aParam : float;
                    vLimit : float; epsilon : float;
                    maxx : float;   maxy : float;
                    minx : float;   miny : float}
    let parms = 
        let maxx = 590.0
        let maxy = 590.0
        let minx = -8.0
        let miny = -8.0
        {velo = 1.02; cParam = 0.06; sParam = 12.0; sScale = 0.2; 
         aParam = 0.16; vLimit = 0.0025 * (max (maxx-minx) (maxy-miny));
         epsilon = 25.0; maxx = maxx; maxy = maxy;
         minx = minx; miny = miny}
    
    let vecZero = 0.0, 0.0

    let inline (<+>) (a, b) (a', b') = a + a', b + b'

    let inline (<->) (a, b) (a', b') = a - a', b - b'

    let inline (</>) (a, b) c = a/c, b/c

    let inline vecScale (s : float) (a, b) = s * a, s * b

    let inline sq x = x * x

    let inline vecNorm (x, y) = sqrt (sq x + sq y)
    //  sometimes we want to control runaway of vector scales, so this can
    // be used to enforce an upper bound
    let inline limiter boidVel speedLimit =
        match boidVel with
        |velX, velY when sq velX + sq velY > sq speedLimit ->
            let slowdown = (sq speedLimit) / (sq velX + sq velY)
            slowdown * velX, slowdown * velY
        |_ -> boidVel

    let inline findCentroid boids =
        match boids with
        | []      -> failwith "Bad centroid"
        | _   -> 
            let average f l = List.averageBy (fun boid -> boid.position |> f) l
            average fst boids, 
                average snd boids
            

// cohesion : go towards centroid.  parameter dictates fraction of
// distance from boid to centroid that contributes to velocity
    let inline cohesion b boids a  = 
        (findCentroid boids) <-> b.position
        |> vecScale a 
    
    //An acceleration to stop us hitting nearby boids
    let inline separation b boids a sScale =
        match boids with
        | [] -> vecZero
        | _ -> 
            boids
            |> List.map (fun boid -> boid.position <-> b.position)
            |> List.filter (fun i -> (vecNorm i) < a)       
            |> List.fold (<->) (0.0,0.0)   
            |> vecScale sScale
    
    //Boids try to match velocity with near boids.
    let inline alignment b boids a  =
        match boids with
        | [] -> vecZero
        | _ ->
            let avrg = List.averageBy (fun boid-> fst boid.velocity ) boids, List.averageBy (fun boid-> snd boid.velocity) boids
            vecScale a (avrg <-> b.velocity) 

    let inline wraparound parms  (x, y)  = 
        let w,h = parms.maxx - parms.minx, parms.maxy - parms.miny        
        let x' = if (x > parms.maxx) then x - w else (if x < parms.minx then x+w else x)     
        let y' = if (y > parms.maxy) then y - h else (if y < parms.miny then y+h else y) 
        (x', y')

    let inline boundPosition (boundMin,boundMax) boid =
        let bound coor =
            match coor > boundMax, coor<boundMin with
            |true,_-> -1.0
            |_,true->1.0
            |_->0.0
        bound <| vecDimSelect boid.position 0, bound <| vecDimSelect boid.position 1

    let inline oneboid parms b boids =  
        let c = cohesion b boids parms.cParam      
        let s = separation b boids parms.sParam parms.sScale 
        let a = alignment b boids parms.aParam    

        //apply rules for current boid.
        let v' =  b.velocity <+> (vecScale 0.3 (c <+> s <+> a))
        match b.bounded with
        | true ->
            let vbound = v' <+> (boundPosition (parms.minx + 8.0, parms.maxx) b)
            let v'' = limiter (vecScale parms.velo vbound) parms.vLimit      
            { b with identifier = b.identifier;  position = b.position <+> v'' ; velocity = v''}
        | false ->            
            let v'' = limiter (vecScale parms.velo v') parms.vLimit      
            let p' =  b.position <+> v''
            { b with identifier = b.identifier;  position = wraparound parms p' ; velocity = v''}

    
    let inline splitBoxHoriz parms (lo, hi, ax, ay) =  
        let (lx, ly), (hx, hy) = lo, hi
        let w = parms.maxx - parms.minx
        if (hx-lx > w)   then 
            [( (parms.minx, ly), (parms.maxx, hy), ax, ay)]  
        else
            if (lx < parms.minx) then 
                [( (parms.minx, ly),  (hx, hy), ax, ay);
                 ( (parms.maxx - (parms.minx - lx), ly), (parms.maxx, hy), (ax - w), ay)]       
            else
                if (hx > parms.maxx)  then 
                    [((lx, ly),  (parms.maxx, hy), ax, ay);
                     ( (parms.minx, ly), (parms.minx + (hx - parms.maxx), hy), ax+w, ay)]            
                else [(lo,hi,ax,ay)]  
    
    let inline splitBoxVert parms (lo, hi, ax, ay) =
        let (lx, ly), (hx, hy) = lo, hi
        let h = parms.maxy - parms.miny
        if (hy-ly > h) then
            [( (lx, parms.miny), (hx, parms.maxy), ax, ay)]
        else 
            if (ly < parms.miny) then
               [((lx, parms.miny),  (hx, hy), ax, ay);
                ((lx, parms.maxy - (parms.miny - ly)), (hx, parms.maxy), ax, ay-h)]
            else 
                if (hy > parms.maxy) then
                    [((lx, ly), (hx, parms.maxy), ax, ay);
                     ((lx, parms.miny), (hx, parms.miny + (hy - parms.maxy)), ax, ay+h)]
                else [(lo,hi,ax,ay)]

    let inline findNeighbors parms tree b =           
        let epsvec = (parms.epsilon, parms.epsilon)
        let vlo, vhi = b.position <-> epsvec,  b.position <+> epsvec            
        
        // adjuster for wraparound      
        let adj1 ax ay (pos, theboid) = 
            (pos <+> (ax,ay), {theboid with position = theboid.position <+> (ax,ay) }) 
        
        let adjuster lo hi ax ay = 
            let neighbors = kdtRangeSearch tree lo hi                             
            List.map (adj1 ax ay) neighbors        
        
        let neighbors =
            match b.bounded with
            | false ->
                //split the boxes      
                let splith = splitBoxHoriz parms (vlo, vhi, 0.0, 0.0)      
                let splitv = List.collect (splitBoxVert parms) splith                      
        
                // do the sequence of range searches 
                List.collect (fun (lo,hi,ax,ay) -> adjuster lo hi ax ay) splitv            
            | true ->
                kdtRangeSearch tree vlo vhi
        // compute the distances from boid b to members 
        let dists = List.map (fun (_, boid) -> (vecNorm (b.position <-> boid.position), boid)) neighbors  
        
        b :: (List.map snd (List.filter (fun (d, _) -> d <= parms.epsilon) dists))


    let inline iterationkd fdraw parms tree =  
        let ftemp f g l = f l, g l
        flatten tree []  
        |> PSeq.map (fun boid -> oneboid parms boid (findNeighbors parms tree boid)) 
        |> ftemp fromListWithDepth (fdraw (int parms.epsilon))              

    let rndm = new Random()

    let inline makeboid i = 
        let x=rndm.NextDouble()
        let y=rndm.NextDouble()
        let m= (float i)%400.0
        match i % 3 with
        | 0 -> 
            {identifier = i; velocity = x - 1.5, y - 1.5; 
             position =  (m * (x - 0.5) , m * (y - 0.5) ); bounded = m > 150.0}
        | 1 -> 
            {identifier = i; velocity = - x , - y; 
             position = m * (x - 0.5) , m * (y - 0.5) + m; bounded = m > 150.0}
        | _ ->
            {identifier = i; velocity = 1.5 - x, y - 1.5; 
             position = m * (x - 0.5) + m, m * (y - 0.5) ; bounded = m > 150.0}