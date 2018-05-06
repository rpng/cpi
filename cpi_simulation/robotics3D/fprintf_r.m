%______________________________________________________________________________
%
% ThyssenKrupp Transrapid                                   Dr. Florian Dignath
% TST - Basic Technologies                                                 v1.0
% Dr. Zheng                                                          08.06.2010
%______________________________________________________________________________
%
% Matlab function for doing a carriage return (without linefeed!)
% followed by some output using fprintf.
%
% This is useful, e.g. to replace the display of a counter i in the style:
% fprintf('\r %f',i), because the \r format is not working properly on windows
% systems.
%
% Input is a string s and a variable x which are forwarded to the
% fprintf command as fprintf(s,x). Use a vector for x in case you want
% to display several variables at once.
%
% In the first call of this function, nothing else is done. Starting
% with the second call, the corresponding number of backspaces is
% inserted before executing fprintf(s,x), so the output starts at the
% place it would have started without the first fprintf command.
%
% If 'reset' is given as the 3rd argument, the script deals with s and
% x as in the first call of the function.
%
% If 'reset' is given as the 1st and only argument, the next call will
% be treated as a "first" call, again.
%
% USAGE: 
%     fprintf_r(s, x[, 'reset']) or
%     fprintf_r('reset')
%
% EXAMPLE:
%     for i = 1:110
%       fprintf_r('%i', i);
%       pause(0.05)
%     end
%     fprintf_r('reset')
%
% N.B: This function is not necessary on UNIX systems, as \r is working fine 
%      there.
%______________________________________________________________________________
%
 function [] = fprintf_r(s,x,res)
%______________________________________________________________________________
%
% global variable for remembering length of last display:
 persistent FPRINTF_R_LENGTH;
%
%------------------------------------------------------------------------------
% check input:
%------------------------------------------------------------------------------
% check number of input arguments:
 nargchk(1, 3, nargin);
%
% check for reset in s:
 if nargin==1
   if strcmp(s,'reset')
     FPRINTF_R_LENGTH = 0;
   return
   else
     error('ERROR: wrong number of input arguments! -> help fprintf_r')
   end
 end
%   
% else check size of s and x:
 [ns,ms] = size(s);
 if ns>1 & ms>1
   error('ERROR: input s must be one string, not a matrix! -> help fprintf_r')
 end
 [nx,mx] = size(x);
 if nx>1 & mx>1
   error('ERROR: input x must be a scalar or vector, not a matrix! -> help fprintf_r')
 end
%
% check for reset in res:
 if nargin==3
   if strcmp(res,'reset')
     FPRINTF_R_LENGTH = 0;
   else
     error('ERROR: 3rd argument may only be "reset"! -> help fprintf_r')
   end
 end
% 
%------------------------------------------------------------------------------
% do backspace if this is not the first call:
%------------------------------------------------------------------------------
 if FPRINTF_R_LENGTH>0
   for j=1:FPRINTF_R_LENGTH
     fprintf('\b');             % backspace
   end
 end
%
%------------------------------------------------------------------------------
% display and store length:
%------------------------------------------------------------------------------
 FPRINTF_R_LENGTH=fprintf(s,x);
%   
% end of function. 
