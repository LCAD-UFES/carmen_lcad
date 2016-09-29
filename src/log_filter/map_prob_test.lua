local function main(vezes)
  local average = 0.0
  vezes = tonumber(vezes)
  for i = 1, vezes do
    local count = 0
    for j = 1, 10 do
      if math.random(1, 6) == 1 then
        count = count + 1
      end
    end
    local prob = count / 10
    average = average + prob 
  end
  print(average / vezes)
end -- main()

main(arg[1])
