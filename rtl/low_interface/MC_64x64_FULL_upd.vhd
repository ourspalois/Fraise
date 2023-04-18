library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;

-- 'Z' = VIRGIN CELL / 'O' = HRS CELL / '1' = LRS CELL 

entity MC_64x64_FULL_upd is
port(	CBL, CBLEN, CSL, DIN, DINb : in std_logic_vector(0 to 63);
	CWLE, CWLO : in std_logic_vector(0 to 31);
	DOUT : out std_logic_vector(0 to 63));
end MC_64x64_FULL_upd;

architecture archi_NEURONIC_4K_FULL_MOD of MC_64x64_FULL_upd is
type t_array is array (0 to 31) of std_logic_vector(0 to 127);
type r_array is array (0 to 31) of std_logic_vector(0 to 63);

signal RRAM_E : t_array := (others => (others => 'Z'));
signal RRAM_O : t_array := (others => (others => 'Z'));

signal RD_EN_E : r_array := (others => (others => '0'));
signal RD_EN_O : r_array := (others => (others => '0'));

signal col      	: natural range 0 to 63;
signal row      	: natural range 0 to 31;

begin

process(CBL, CBLEN, CSL, DIN, DINb, CWLE, CWLO)
begin
	for row in 0 to 31 loop

-- Gestion des lignes even
		if (CWLE(row) = '1') then
			for col in 0 to 63 loop
-- ecriture even
				if (CBLEN(col) = '1') then
					if    (CBL(col) = '1' and CSL(col) = '0') then
						RRAM_E(row)(2*col) <= '0';
					elsif (CBL(col) = '0' and CSL(col) = '0') then
						RRAM_E(row)(2*col+1) <= '0';
					elsif (CBL(col) = '1' and CSL(col) = '1') then
						RRAM_E(row)(2*col+1) <= '1';
					elsif (CBL(col) = '0' and CSL(col) = '1') then
						RRAM_E(row)(2*col) <= '1';
					end if;
					RD_EN_E(row)(col) <= '0';
				else
-- lecture even
					if (CSL(col) = '0' and RD_EN_E(row)(col) = '1') then
						if (((CWLO = X"0000") and (CWLE = X"0000")) or ((DIN = X"00000000") and (DINb = X"00000000")))then
							DOUT <= "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
						else 
							if (RRAM_E(row)(2*col) = RRAM_E(row)(2*col+1)) then
								DOUT <= "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
								RD_EN_E(row)(col) <= '0';
							else
								DOUT(col) <= not((RRAM_E(row)(2*col) and DIN(col)) or (RRAM_E(row)(2*col+1) and DINb(col)));
								RD_EN_E(row)(col) <= '0';
							end if;
						end if;
					elsif (CSL(col) = '1') then
						RD_EN_E(row)(col) <= '1';
					end if;
				end if;
			end loop;
		end if;
		
-- Gestion des lignes odd
		if (CWLO(row) = '1') then
			for col in 0 to 63 loop
-- ecriture odd
				if (CBLEN(col) = '1') then
					if    (CBL(col) = '1' and CSL(col) = '0') then
						RRAM_O(row)(2*col) <= '0';
					elsif (CBL(col) = '0' and CSL(col) = '0') then
						RRAM_O(row)(2*col+1) <= '0';
					elsif (CBL(col) = '1' and CSL(col) = '1') then
						RRAM_O(row)(2*col+1) <= '1';
					elsif (CBL(col) = '0' and CSL(col) = '1') then
						RRAM_O(row)(2*col) <= '1';
					end if;
					RD_EN_O(row)(col) <= '0';
				else
-- lecture odd
					if (CSL(col) = '0' and RD_EN_O(row)(col) = '1') then
						if (((CWLO = X"0000") and (CWLE = X"0000")) or ((DIN = X"00000000") and (DINb = X"00000000")))then
							DOUT <= "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
						else 
							if (RRAM_O(row)(2*col) = RRAM_O(row)(2*col+1)) then
								DOUT <= "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
								RD_EN_O(row)(col) <= '0';
							else
								DOUT(col) <= not((RRAM_O(row)(2*col) and DIN(col)) or (RRAM_O(row)(2*col+1) and DINb(col)));
								RD_EN_O(row)(col) <= '0';
							end if;
						end if;
					elsif (CSL(col) = '1') then
						RD_EN_O(row)(col) <= '1';
					end if;
				end if;
			end loop;
		end if;

	end loop;
	
end process;


end;

