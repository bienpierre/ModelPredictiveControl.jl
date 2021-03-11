using Documenter, EasyMPC

makedocs(

    sitename="EasyMPC.jl",
    
    pages=[
        "Home" => "index.md",
        "Problems" => Any["linearmpc.md"],
        "Examples" => Any["Examples/QTP.md"],
        "Functions" => Any[
                        "functions.md"],
         "Style guide" => Any["style_guide.md"]               
        
    ] )
