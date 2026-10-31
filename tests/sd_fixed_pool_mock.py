"""Finite ThreadX block-pool mock for production SD admission tests.

Charges the native ABI's rounded payload plus one link pointer per block, as
_tx_block_pool_create does. Arrays are real bounded backing storage; allocation
never grows a heap. A four-byte offset stresses the MCU's word-only alignment.
This verifies capacity/ownership, not ThreadX execution time.
"""
TYPES = r'''
typedef struct {
 size_t tx_block_pool_block_size,tx_block_pool_total,tx_block_pool_available,budget;
 unsigned char *storage; unsigned occupied[1024];
} TX_BLOCK_POOL;
'''
FUNCTIONS = r'''
static TX_BLOCK_POOL *mock_pools[2];
static unsigned mock_pool_count,mock_force_full,mock_allocations_count;
static void mock_pool_init(TX_BLOCK_POOL *pool,unsigned char *storage,size_t budget,size_t requested){
 assert(mock_pool_count<2);
 pool->tx_block_pool_block_size=(requested+sizeof(void*)-1)/sizeof(void*)*sizeof(void*);
 pool->budget=budget/sizeof(void*)*sizeof(void*);
 pool->tx_block_pool_total=pool->budget/(pool->tx_block_pool_block_size+sizeof(void*));
 assert(pool->tx_block_pool_total&&pool->tx_block_pool_total<=1024);
 pool->tx_block_pool_available=pool->tx_block_pool_total;
 pool->storage=storage+4; /* test eight-byte alignment above word-aligned memory */
 mock_pools[mock_pool_count++]=pool;
}
static unsigned tx_block_allocate(TX_BLOCK_POOL *pool,void **out,unsigned wait){
 assert(!irq_mask&&wait==TX_NO_WAIT);
 if(mock_force_full||!pool->tx_block_pool_available)return 1;
 for(size_t i=0;i<pool->tx_block_pool_total;i++)if(!pool->occupied[i]){
  pool->occupied[i]=1;--pool->tx_block_pool_available;++mock_allocations_count;
  *out=pool->storage+i*(pool->tx_block_pool_block_size+sizeof(void*))+sizeof(void*);return TX_SUCCESS;
 }
 assert(0);return 1;
}
static unsigned tx_block_release(void *memory){
 assert(!irq_mask);
 for(unsigned n=0;n<mock_pool_count;n++){
  TX_BLOCK_POOL *pool=mock_pools[n];
  for(size_t i=0;i<pool->tx_block_pool_total;i++){
   void *expected=pool->storage+i*(pool->tx_block_pool_block_size+sizeof(void*))+sizeof(void*);
   if(expected==memory){assert(pool->occupied[i]&&pool->tx_block_pool_available<pool->tx_block_pool_total);
    pool->occupied[i]=0;++pool->tx_block_pool_available;return TX_SUCCESS;}
  }
 }
 assert(0);return 1;
}
static size_t mock_pool_used_bytes(const TX_BLOCK_POOL *pool){
 return (pool->tx_block_pool_total-pool->tx_block_pool_available)*(pool->tx_block_pool_block_size+sizeof(void*));
}
'''
