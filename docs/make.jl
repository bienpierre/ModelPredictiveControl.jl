using Documenter, ModelPredictiveControl

makedocs(
    modules=[ModelPredictiveControl],
    format = Documenter.HTML(),
    sitename="ModelPredictiveControl.jl",
    
    pages=[
        "Home" => "index.md",
        "Problems" => Any["linearmpc.md"],
        "Tools" => Any["tools.md"], 
        "Examples" => Any["Examples/QTP.md",
                          "Examples/mass-string-damper.md"],
        "Functions" => Any[
                        "functions.md"],
        "Style guide" => Any["style_guide.md"],
                       
        
    ] )
