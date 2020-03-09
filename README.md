# Marvin
Marvin is an injection based bot for Continuum.  

It injects a dll into the Continuum process and uses Detours to override GetAsyncKeyState and PeekMessage. It simulates input by changing GetAsyncKeyState to only return the inputs that it desires. It overrides PeekMessage so it has a place to inject itself into the main update loop.  
  
This method can be combined with multicont to run a bunch of bots with little overhead.  
  
## Shortcuts
`F9`: Enables the bot and takes control of the key presses.  
`F10`: Disables the bot and allows the user to control the client again.  
