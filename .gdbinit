define hook-step 
mon cortex_m maskisr on 
end

define hookpost-step 
mon cortex_m maskisr off 
end 
